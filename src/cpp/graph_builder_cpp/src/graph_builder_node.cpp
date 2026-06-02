// graph_builder_node.cpp — Interactive route graph builder for nav2_route.
//
// Listens for Publish Point clicks in RViz, builds a GeoJSON waypoint graph
// in memory, visualises it live as RViz markers, and saves/loads on demand.
//
// RViz setup:
//   1. Add → MarkerArray → topic /graph_builder/markers
//   2. Select "Publish Point" tool (crosshair in toolbar, shortcut: g)
//   3. Click on the map to place nodes
//
// Commands via /graph_builder/cmd (std_msgs/msg/String):
//   save [path]          save graph (default: save_path param)
//   load <path>          load and display existing GeoJSON
//   connect <a> <b>      bidirectional edge between nodes a and b
//   connect_one <a> <b>  directed edge a -> b only
//   disconnect <a> <b>   remove edge between a and b
//   delete <id>          remove node and all attached edges
//   undo                 remove last added node or edge
//   clear                remove everything
//   autoconnect          add edges between all node pairs with clear line-of-sight
//   auto on|off          toggle auto-connect on new node (default: on)
//   name <id> <text>     set display name for node id
//   info                 print summary to terminal
//
// Parameters (ros2 param set /graph_builder <name> <value>):
//   save_path    (string)   output GeoJSON file  [~/maps/graph.geojson]
//   auto_connect (bool)     chain-connect new nodes  [true]
//   map_frame    (string)   coordinate frame for markers  [map]
//
// Usage:
//   ros2 run graph_builder_cpp graph_builder
//       --ros-args -p save_path:=/home/vmx/maps/court_graph.geojson

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using json = nlohmann::json;
namespace fs = std::filesystem;

using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker      = visualization_msgs::msg::Marker;
using Point       = geometry_msgs::msg::Point;
using ColorRGBA   = std_msgs::msg::ColorRGBA;

static ColorRGBA rgba(float r, float g, float b, float a = 1.0f)
{
    ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

static Point pt(double x, double y, double z = 0.0)
{
    Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

// Split string on whitespace.
static std::vector<std::string> tokenise(const std::string & s)
{
    std::istringstream iss(s);
    std::vector<std::string> out;
    std::string tok;
    while (iss >> tok) { out.push_back(tok); }
    return out;
}

struct GraphNode {
    int         id;
    double      x, y;
    std::string name;
};

struct GraphEdge {
    int  id, start, end;
    bool bidirectional;
    bool auto_connected {false};   // true = added by autoconnect, shown cyan in RViz
};

enum class UndoKind { Node, Edge };
struct UndoEntry { UndoKind kind; int id; };

class GraphBuilderNode : public rclcpp::Node
{
public:
    explicit GraphBuilderNode()
    : Node("graph_builder")
    {
        // Build a sensible default save path: ~/maps/graph.geojson
        const char * home_env = std::getenv("HOME");
        std::string default_path = home_env
            ? (fs::path(home_env) / "maps" / "graph.geojson").string()
            : "/tmp/graph.geojson";

        declare_parameter("save_path",    default_path);
        declare_parameter("auto_connect", true);
        declare_parameter("map_frame",    std::string("map"));

        pub_markers_ = create_publisher<MarkerArray>("/graph_builder/markers", 10);

        sub_point_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg){ onClickedPoint(msg); });

        sub_cmd_ = create_subscription<std_msgs::msg::String>(
            "/graph_builder/cmd", 10,
            [this](std_msgs::msg::String::SharedPtr msg){ onCommand(msg); });

        // Transient-local matches map_server's QoS — receives the map even if subscribed late.
        sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                map_ = msg;
                RCLCPP_INFO(get_logger(), "Map received: %ux%u cells @ %.3f m/cell",
                    msg->info.width, msg->info.height, msg->info.resolution);
            });

        // Refresh markers at 2 Hz so RViz stays current even between edits
        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this](){ publishMarkers(); });

        RCLCPP_INFO(get_logger(), "Graph builder ready.");
        RCLCPP_INFO(get_logger(), "  save_path   = %s", get_parameter("save_path").as_string().c_str());
        RCLCPP_INFO(get_logger(), "  auto_connect = %s",
            get_parameter("auto_connect").as_bool() ? "true" : "false");
        RCLCPP_INFO(get_logger(), "Use 'Publish Point' (g) in RViz to place nodes.");
        RCLCPP_INFO(get_logger(), "Commands on /graph_builder/cmd  e.g: \"save\", \"connect 0 3\", \"undo\"");
    }

private:
    std::map<int, GraphNode> nodes_;
    std::map<int, GraphEdge> edges_;
    int                      next_node_id_ {0};
    int                      next_edge_id_ {1000};   // keep IDs visually distinct
    std::optional<int>       last_node_id_;
    std::vector<UndoEntry>   undo_stack_;

    rclcpp::Publisher<MarkerArray>::SharedPtr                              pub_markers_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr     sub_point_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                 sub_cmd_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr         sub_map_;
    rclcpp::TimerBase::SharedPtr                                           timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;   // latest map for autoconnect raytrace

    static constexpr float       NODE_RADIUS       = 0.08f;    // metres
    static constexpr float       EDGE_WIDTH        = 0.03f;
    static constexpr float       LABEL_HEIGHT      = 0.12f;
    static constexpr const char* NS_NODE           = "nodes";
    static constexpr const char* NS_EDGE           = "edges";
    static constexpr const char* NS_LABEL          = "labels";

    // Half-width of robot (metres) — used for line-of-sight clearance in autoconnect.
    // Raytrace rejects edges where any cell within this distance of the line is occupied.
    static constexpr double ROBOT_HALF_WIDTH = 0.195;   // 390mm robot / 2

    // Clicked point — place a new node
    void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr & msg)
    {
        const int    id = next_node_id_++;
        const double x  = msg->point.x;
        const double y  = msg->point.y;

        nodes_[id] = { id, x, y, "wp_" + std::to_string(id) };
        undo_stack_.push_back({ UndoKind::Node, id });

        RCLCPP_INFO(get_logger(), "Added node %d  (%.3f, %.3f)", id, x, y);

        if (get_parameter("auto_connect").as_bool() && last_node_id_.has_value()) {
            addEdge(*last_node_id_, id, /*bidirectional=*/true);
        }
        last_node_id_ = id;
        publishMarkers();
    }

    // Command dispatch
    void onCommand(const std_msgs::msg::String::SharedPtr & msg)
    {
        auto toks = tokenise(msg->data);
        if (toks.empty()) { return; }

        const auto & cmd = toks[0];

        if (cmd == "save") {
            std::string path = (toks.size() > 1)
                ? toks[1]
                : get_parameter("save_path").as_string();
            saveGraph(path);

        } else if (cmd == "load") {
            if (toks.size() < 2) { RCLCPP_ERROR(get_logger(), "load: path required"); return; }
            loadGraph(toks[1]);

        } else if (cmd == "connect") {
            if (toks.size() < 3) { RCLCPP_ERROR(get_logger(), "connect: need two node IDs"); return; }
            tryParseAndConnect(toks[1], toks[2], /*bidi=*/true);

        } else if (cmd == "connect_one") {
            if (toks.size() < 3) { RCLCPP_ERROR(get_logger(), "connect_one: need two node IDs"); return; }
            tryParseAndConnect(toks[1], toks[2], /*bidi=*/false);

        } else if (cmd == "disconnect") {
            if (toks.size() < 3) { RCLCPP_ERROR(get_logger(), "disconnect: need two node IDs"); return; }
            try { removeEdgeBetween(std::stoi(toks[1]), std::stoi(toks[2])); }
            catch (...) { RCLCPP_ERROR(get_logger(), "disconnect: invalid IDs"); return; }

        } else if (cmd == "delete") {
            if (toks.size() < 2) { RCLCPP_ERROR(get_logger(), "delete: need a node ID"); return; }
            try { deleteNode(std::stoi(toks[1])); }
            catch (...) { RCLCPP_ERROR(get_logger(), "delete: invalid ID"); return; }

        } else if (cmd == "undo") {
            undoLast();

        } else if (cmd == "clear") {
            nodes_.clear();
            edges_.clear();
            undo_stack_.clear();
            last_node_id_.reset();
            next_node_id_ = 0;
            next_edge_id_ = 1000;
            RCLCPP_INFO(get_logger(), "Graph cleared.");

        } else if (cmd == "autoconnect") {
            autoConnect();

        } else if (cmd == "auto") {
            if (toks.size() < 2) { RCLCPP_ERROR(get_logger(), "auto: need on|off"); return; }
            bool val = (toks[1] == "on");
            set_parameter(rclcpp::Parameter("auto_connect", val));
            RCLCPP_INFO(get_logger(), "auto_connect = %s", val ? "true" : "false");

        } else if (cmd == "name") {
            if (toks.size() < 3) { RCLCPP_ERROR(get_logger(), "name: need <id> <text>"); return; }
            try {
                int nid = std::stoi(toks[1]);
                if (!nodes_.count(nid)) {
                    RCLCPP_ERROR(get_logger(), "name: node %d not found", nid); return;
                }
                std::string newname;
                for (size_t i = 2; i < toks.size(); ++i) {
                    if (i > 2) newname += ' ';
                    newname += toks[i];
                }
                nodes_[nid].name = newname;
                RCLCPP_INFO(get_logger(), "Node %d → \"%s\"", nid, newname.c_str());
            } catch (...) { RCLCPP_ERROR(get_logger(), "name: invalid ID"); return; }

        } else if (cmd == "info") {
            RCLCPP_INFO(get_logger(), "Graph: %zu nodes, %zu edges",
                nodes_.size(), edges_.size());
            for (auto & [id, n] : nodes_) {
                RCLCPP_INFO(get_logger(), "  node %3d  (%7.3f, %7.3f)  \"%s\"",
                    id, n.x, n.y, n.name.c_str());
            }
            for (auto & [id, e] : edges_) {
                RCLCPP_INFO(get_logger(), "  edge %4d  %d %s %d",
                    id, e.start, e.bidirectional ? "<->" : "->", e.end);
            }

        } else {
            RCLCPP_WARN(get_logger(), "Unknown command: %s  "
                "(save|load|connect|connect_one|disconnect|delete|undo|clear|autoconnect|auto|name|info)",
                cmd.c_str());
        }

        publishMarkers();
    }

    // Graph operations
    void tryParseAndConnect(const std::string & a_str, const std::string & b_str, bool bidi)
    {
        try { addEdge(std::stoi(a_str), std::stoi(b_str), bidi); }
        catch (...) { RCLCPP_ERROR(get_logger(), "connect: invalid node IDs"); }
    }

    void addEdge(int a, int b, bool bidi)
    {
        if (!nodes_.count(a)) {
            RCLCPP_ERROR(get_logger(), "addEdge: node %d not found", a); return;
        }
        if (!nodes_.count(b)) {
            RCLCPP_ERROR(get_logger(), "addEdge: node %d not found", b); return;
        }
        // Duplicate check
        for (auto & [eid, e] : edges_) {
            if ((e.start == a && e.end == b) ||
                (e.bidirectional && e.start == b && e.end == a)) {
                RCLCPP_WARN(get_logger(), "Edge %d<->%d already exists — skipping", a, b);
                return;
            }
        }
        const int eid = next_edge_id_++;
        edges_[eid] = { eid, a, b, bidi };
        undo_stack_.push_back({ UndoKind::Edge, eid });
        RCLCPP_INFO(get_logger(), "Added edge %d  %d %s %d",
            eid, a, bidi ? "<->" : "->", b);
    }

    void removeEdgeBetween(int a, int b)
    {
        std::vector<int> to_del;
        for (auto & [eid, e] : edges_) {
            if ((e.start == a && e.end == b) || (e.start == b && e.end == a))
                to_del.push_back(eid);
        }
        for (int eid : to_del) { edges_.erase(eid); }
        if (to_del.empty())
            RCLCPP_WARN(get_logger(), "No edge found between %d and %d", a, b);
        else
            for (int eid : to_del)
                RCLCPP_INFO(get_logger(), "Removed edge %d", eid);
    }

    void deleteNode(int nid)
    {
        if (!nodes_.count(nid)) {
            RCLCPP_ERROR(get_logger(), "deleteNode: node %d not found", nid); return;
        }
        nodes_.erase(nid);
        std::vector<int> to_del;
        for (auto & [eid, e] : edges_) {
            if (e.start == nid || e.end == nid) to_del.push_back(eid);
        }
        for (int eid : to_del) { edges_.erase(eid); }
        if (last_node_id_.has_value() && *last_node_id_ == nid) last_node_id_.reset();
        RCLCPP_INFO(get_logger(), "Deleted node %d and %zu attached edge(s)",
            nid, to_del.size());
    }

    void undoLast()
    {
        if (undo_stack_.empty()) { RCLCPP_WARN(get_logger(), "Nothing to undo"); return; }

        auto [kind, id] = undo_stack_.back();
        undo_stack_.pop_back();

        if (kind == UndoKind::Node) {
            if (nodes_.count(id)) {
                nodes_.erase(id);
                // Remove dangling edges
                std::vector<int> to_del;
                for (auto & [eid, e] : edges_)
                    if (e.start == id || e.end == id) to_del.push_back(eid);
                for (int eid : to_del) { edges_.erase(eid); }
            }
            // Walk stack back to find the new "last" node
            last_node_id_.reset();
            for (auto it = undo_stack_.rbegin(); it != undo_stack_.rend(); ++it) {
                if (it->kind == UndoKind::Node && nodes_.count(it->id)) {
                    last_node_id_ = it->id;
                    break;
                }
            }
            RCLCPP_INFO(get_logger(), "Undo: removed node %d", id);

        } else {
            if (edges_.count(id)) { edges_.erase(id); }
            RCLCPP_INFO(get_logger(), "Undo: removed edge %d", id);
        }
    }

    // Save / Load
    void saveGraph(const std::string & path)
    {
        try {
            auto parent = fs::path(path).parent_path();
            if (!parent.empty()) { fs::create_directories(parent); }

            json features = json::array();

            for (auto & [id, n] : nodes_) {
                features.push_back({
                    {"type",     "Feature"},
                    {"geometry", {
                        {"type",        "Point"},
                        {"coordinates", {n.x, n.y}}
                    }},
                    {"properties", {
                        {"id",    n.id},
                        {"frame", "map"},
                        {"name",  n.name}
                    }}
                });
            }

            // Bidirectional edges emit two directed features (nav2_route format).
            // Reverse edges get IDs starting after the highest existing edge ID.
            int rev_eid = next_edge_id_;
            for (auto & [id, e] : edges_) {
                auto & sn = nodes_.at(e.start);
                auto & en = nodes_.at(e.end);
                features.push_back({
                    {"type",     "Feature"},
                    {"geometry", {
                        {"type",        "MultiLineString"},
                        {"coordinates", {{{sn.x, sn.y}, {en.x, en.y}}}}
                    }},
                    {"properties", {
                        {"id",      e.id},
                        {"startid", e.start},
                        {"endid",   e.end}
                    }}
                });
                if (e.bidirectional) {
                    features.push_back({
                        {"type",     "Feature"},
                        {"geometry", {
                            {"type",        "MultiLineString"},
                            {"coordinates", {{{en.x, en.y}, {sn.x, sn.y}}}}
                        }},
                        {"properties", {
                            {"id",      rev_eid++},
                            {"startid", e.end},
                            {"endid",   e.start}
                        }}
                    });
                }
            }

            json geojson = {{"type", "FeatureCollection"}, {"features", features}};

            std::ofstream f(path);
            if (!f.is_open()) {
                RCLCPP_ERROR(get_logger(), "Cannot open %s for writing", path.c_str());
                return;
            }
            f << geojson.dump(2);
            RCLCPP_INFO(get_logger(), "Saved %zu nodes, %zu edges → %s",
                nodes_.size(), edges_.size(), path.c_str());

        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "saveGraph error: %s", e.what());
        }
    }

    void loadGraph(const std::string & path)
    {
        if (!fs::exists(path)) {
            RCLCPP_ERROR(get_logger(), "File not found: %s", path.c_str());
            return;
        }

        try {
            std::ifstream f(path);
            json data;
            f >> data;

            nodes_.clear();
            edges_.clear();
            undo_stack_.clear();
            last_node_id_.reset();

            for (auto & feat : data["features"]) {
                const std::string gt = feat["geometry"]["type"];
                auto & props  = feat["properties"];
                auto & coords = feat["geometry"]["coordinates"];

                if (gt == "Point") {
                    // Support both old format (nodeid) and nav2_route format (id in props)
                    int nid = props.contains("nodeid") ? props["nodeid"].get<int>()
                                                       : props["id"].get<int>();
                    double x = coords[0];
                    double y = coords[1];
                    std::string nm = props.value("name", "wp_" + std::to_string(nid));
                    nodes_[nid] = { nid, x, y, nm };
                    if (nid >= next_node_id_) next_node_id_ = nid + 1;

                } else if (gt == "LineString") {
                    // Old format: one feature per edge, bidirectional flag in props
                    int  eid   = props["edgeid"].get<int>();
                    int  start = props["startid"].get<int>();
                    int  end   = props["endid"].get<int>();
                    bool bidi  = props.value("bidirectional", true);
                    edges_[eid] = { eid, start, end, bidi };
                    if (eid >= next_edge_id_) next_edge_id_ = eid + 1;

                } else if (gt == "MultiLineString") {
                    // nav2_route format: two directed features per bidirectional edge.
                    // When the reverse of an already-loaded edge arrives, merge it.
                    int eid   = props["id"].get<int>();
                    int start = props["startid"].get<int>();
                    int end   = props["endid"].get<int>();
                    bool merged = false;
                    for (auto & [k, ex] : edges_) {
                        if (ex.start == end && ex.end == start) {
                            ex.bidirectional = true;
                            merged = true;
                            break;
                        }
                    }
                    if (!merged) {
                        edges_[eid] = { eid, start, end, false };
                        if (eid >= next_edge_id_) next_edge_id_ = eid + 1;
                    }
                }
            }

            if (!nodes_.empty()) {
                last_node_id_ = nodes_.rbegin()->first;
            }

            RCLCPP_INFO(get_logger(), "Loaded %zu nodes, %zu edges from %s",
                nodes_.size(), edges_.size(), path.c_str());

        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "loadGraph error: %s", e.what());
        }
    }

    // Line-of-sight raytrace (used by autoConnect)
    // Returns true if a straight line from (x0,y0) to (x1,y1) has no occupied or
    // unknown cells within ROBOT_HALF_WIDTH metres of the line, using Bresenham's
    // algorithm on the current occupancy map.
    bool lineIsClear(double x0, double y0, double x1, double y1) const
    {
        const auto & info = map_->info;
        const double res  = info.resolution;
        const double ox   = info.origin.position.x;
        const double oy   = info.origin.position.y;
        const int    W    = static_cast<int>(info.width);
        const int    H    = static_cast<int>(info.height);
        // Clearance in grid cells — how many cells each side of the line to check.
        const int clr = std::max(1, static_cast<int>(std::ceil(ROBOT_HALF_WIDTH / res)));

        int cx0 = static_cast<int>((x0 - ox) / res);
        int cy0 = static_cast<int>((y0 - oy) / res);
        int cx1 = static_cast<int>((x1 - ox) / res);
        int cy1 = static_cast<int>((y1 - oy) / res);

        auto cellBlocked = [&](int gx, int gy) -> bool {
            if (gx < 0 || gy < 0 || gx >= W || gy >= H) return true;
            const int8_t v = map_->data[gy * W + gx];
            return (v < 0 || v > 50);   // unknown (-1) or occupied (>50)
        };

        int dx  = std::abs(cx1 - cx0);
        int dy  = std::abs(cy1 - cy0);
        int sx  = (cx0 < cx1) ? 1 : -1;
        int sy  = (cy0 < cy1) ? 1 : -1;
        int err = dx - dy;
        int cx  = cx0;
        int cy  = cy0;

        while (true) {
            // Check a square clearance neighborhood around each step.
            for (int nx = cx - clr; nx <= cx + clr; ++nx)
                for (int ny = cy - clr; ny <= cy + clr; ++ny)
                    if (cellBlocked(nx, ny)) return false;

            if (cx == cx1 && cy == cy1) break;

            const int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; cx += sx; }
            if (e2 <  dx) { err += dx; cy += sy; }
        }
        return true;
    }

    // Auto-connect: add edges between all node pairs with clear line-of-sight
    void autoConnect()
    {
        if (!map_) {
            RCLCPP_ERROR(get_logger(),
                "autoconnect: no map received yet — is map_server (or slam_toolbox) running?");
            return;
        }
        if (nodes_.size() < 2) {
            RCLCPP_WARN(get_logger(), "autoconnect: need at least 2 nodes");
            return;
        }

        std::vector<int> ids;
        ids.reserve(nodes_.size());
        for (auto & [id, n] : nodes_) ids.push_back(id);

        int added = 0, blocked = 0, existed = 0;

        for (size_t i = 0; i < ids.size(); ++i) {
            for (size_t j = i + 1; j < ids.size(); ++j) {
                const int a = ids[i];
                const int b = ids[j];

                // Skip if an edge already exists in either direction
                bool exists = false;
                for (auto & [eid, e] : edges_) {
                    if ((e.start == a && e.end == b) || (e.start == b && e.end == a)) {
                        exists = true; break;
                    }
                }
                if (exists) { ++existed; continue; }

                const auto & na = nodes_.at(a);
                const auto & nb = nodes_.at(b);
                const double dist = std::hypot(nb.x - na.x, nb.y - na.y);

                if (lineIsClear(na.x, na.y, nb.x, nb.y)) {
                    const int eid = next_edge_id_++;
                    edges_[eid] = { eid, a, b, /*bidirectional=*/true, /*auto_connected=*/true };
                    undo_stack_.push_back({ UndoKind::Edge, eid });
                    RCLCPP_INFO(get_logger(), "  auto-edge %d: %d <-> %d  (%.2f m)", eid, a, b, dist);
                    ++added;
                } else {
                    ++blocked;
                }
            }
        }

        RCLCPP_INFO(get_logger(),
            "autoconnect: +%d edges added  |  %d blocked by obstacles  |  %d already existed",
            added, blocked, existed);
    }

    // Marker publishing
    // Build a partially-initialised marker for a given namespace and type.
    Marker makeBase(const std::string & ns, int id, int type)
    {
        Marker m;
        m.header.frame_id     = get_parameter("map_frame").as_string();
        m.header.stamp        = now();
        m.ns                  = ns;
        m.id                  = id;
        m.type                = type;
        m.action              = Marker::ADD;
        m.pose.orientation.w  = 1.0;
        m.lifetime.sec        = 0;    // 0 = keep until replaced or DELETEALL
        m.lifetime.nanosec    = 0;
        return m;
    }

    void publishMarkers()
    {
        MarkerArray ma;

        // Clear every existing marker before re-drawing
        {
            Marker del;
            del.action = Marker::DELETEALL;
            ma.markers.push_back(del);
        }

        for (auto & [id, n] : nodes_) {
            const bool is_last = (last_node_id_.has_value() && *last_node_id_ == id);

            // Sphere
            auto m = makeBase(NS_NODE, id, Marker::SPHERE);
            m.pose.position.x   = n.x;
            m.pose.position.y   = n.y;
            m.pose.position.z   = 0.05;
            m.scale.x = m.scale.y = m.scale.z = NODE_RADIUS * 2.0f;
            m.color = is_last ? rgba(1.f, 0.6f, 0.1f) : rgba(0.2f, 0.5f, 1.f);
            ma.markers.push_back(m);

            // Label above sphere
            auto lm = makeBase(NS_LABEL, id, Marker::TEXT_VIEW_FACING);
            lm.pose.position.x = n.x;
            lm.pose.position.y = n.y;
            lm.pose.position.z = NODE_RADIUS + 0.08f;
            lm.scale.z         = LABEL_HEIGHT;
            lm.color           = rgba(1.f, 1.f, 1.f);
            lm.text            = std::to_string(id) + ": " + n.name;
            ma.markers.push_back(lm);
        }

        for (auto & [eid, e] : edges_) {
            auto it_s = nodes_.find(e.start);
            auto it_e = nodes_.find(e.end);
            if (it_s == nodes_.end() || it_e == nodes_.end()) { continue; }

            const auto & sn = it_s->second;
            const auto & en = it_e->second;

            ColorRGBA col = e.auto_connected
                ? rgba(0.0f, 0.9f, 0.9f, 0.75f)   // cyan  = auto line-of-sight edge
                : (e.bidirectional
                    ? rgba(0.2f, 0.9f, 0.3f, 0.85f)   // green  = manual bidirectional
                    : rgba(1.0f, 0.9f, 0.1f, 0.85f));  // yellow = manual one-way

            auto m = makeBase(NS_EDGE, eid, Marker::LINE_STRIP);
            m.scale.x = EDGE_WIDTH;
            m.color   = col;
            m.points  = { pt(sn.x, sn.y, 0.03), pt(en.x, en.y, 0.03) };
            ma.markers.push_back(m);

            // Small direction chevron mid-edge for one-way edges
            if (!e.bidirectional) {
                const double mx  = (sn.x + en.x) * 0.5;
                const double my  = (sn.y + en.y) * 0.5;
                const double dx  = en.x - sn.x;
                const double dy  = en.y - sn.y;
                const double len = std::hypot(dx, dy);
                if (len > 0.01) {
                    const double nx = -dy / len * 0.06;
                    const double ny =  dx / len * 0.06;
                    auto arrow = makeBase(NS_EDGE, eid + 10000, Marker::LINE_STRIP);
                    arrow.scale.x = EDGE_WIDTH * 0.6f;
                    arrow.color   = col;
                    arrow.points  = {
                        pt(mx + nx,             my + ny,             0.04),
                        pt(mx,                  my,                  0.04),
                        pt(mx + dx/len * 0.08,  my + dy/len * 0.08, 0.04)
                    };
                    ma.markers.push_back(arrow);
                }
            }
        }

        pub_markers_->publish(ma);
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphBuilderNode>());
    rclcpp::shutdown();
    return 0;
}
