#include <bits/stdc++.h>
using namespace std;

struct Edge {
    char to;
    double w;
};

struct Graph {
    unordered_map<char, vector<Edge>> adj;

    void add_edge(char u, char v, double w) {
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
    }

    const vector<Edge>& neighbors(char u) const {
        static const vector<Edge> empty;
        auto it = adj.find(u);
        if (it == adj.end()) return empty;
        return it->second;
    }

    vector<char> nodes() const {
        vector<char> v;
        v.reserve(adj.size());
        for (auto &p : adj) v.push_back(p.first);
        sort(v.begin(), v.end());
        return v;
    }
};

struct PathResult {
    bool found = false;
    vector<char> path;
    double cost = numeric_limits<double>::infinity();
    string name;
};

static inline string path_to_string(const vector<char>& p) {
    string s;
    for (size_t i = 0; i < p.size(); ++i) {
        if (i) s += "->";
        s += p[i];
    }
    return s;
}

static inline double heuristic(char n, char /*goal*/) {
    return double(((n - 'A') * 37) % 13 + 1);
}

static vector<char> reconstruct(char start, char goal, const unordered_map<char,char>& parent) {
    vector<char> path;
    char cur = goal;
    while (true) {
        path.push_back(cur);
        if (cur == start) break;
        auto it = parent.find(cur);
        if (it == parent.end()) { path.clear(); return path; }
        cur = it->second;
    }
    reverse(path.begin(), path.end());
    return path;
}


struct TieBreaker {
    long long counter = 0;
    long long next() { return ++counter; }
};


PathResult ORACLE(const Graph& g, char start, char goal) {
    struct Node { double g; char v; long long t; };
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            if (a.g != b.g) return a.g > b.g;
            if (a.t != b.t) return a.t > b.t;
            return a.v > b.v;
        }
    };
    priority_queue<Node, vector<Node>, Cmp> pq;
    unordered_map<char, double> dist;
    unordered_map<char, char> parent;
    TieBreaker tb;

    for (auto v : g.nodes()) dist[v] = numeric_limits<double>::infinity();
    dist[start] = 0.0;
    pq.push({0.0, start, tb.next()});

    while (!pq.empty()) {
        auto [dg, u, t] = pq.top(); pq.pop();
        if (dg != dist[u]) continue;
        if (u == goal) {
            auto path = reconstruct(start, goal, parent);
            return {true, path, dist[u], "ORACLE"};
        }
        for (const auto &e : g.neighbors(u)) {
            double nd = dg + e.w;
            if (nd < dist[e.to] - 1e-12) {
                dist[e.to] = nd;
                parent[e.to] = u;
                pq.push({nd, e.to, tb.next()});
            } else if (fabs(nd - dist[e.to]) <= 1e-12) {
                if (!parent.count(e.to) || u < parent[e.to]) {
                    parent[e.to] = u;
                    pq.push({nd, e.to, tb.next()});
                }
            }
        }
    }
    return {false, {}, numeric_limits<double>::infinity(), "ORACLE"};
}


PathResult BritishMuseumSearch(const Graph& g, char start, char goal) {
    PathResult best; best.name = "BMS";
    vector<char> path; path.push_back(start);
    unordered_set<char> onpath; onpath.insert(start);
    double bestCost = numeric_limits<double>::infinity();
    double curCost = 0.0;

    function<void(char)> dfs = [&](char u) {
        if (u == goal) {
            if (curCost < bestCost - 1e-12 ||
                (fabs(curCost - bestCost) <= 1e-12 && (best.path.empty() || path < best.path))) {
                bestCost = curCost;
                best.found = true;
                best.path = path;
                best.cost = curCost;
            }
            return;
        }
        vector<Edge> nbrs = g.neighbors(u);
        sort(nbrs.begin(), nbrs.end(), [](const Edge& a, const Edge& b){
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        for (const auto &e : nbrs) {
            if (onpath.count(e.to)) continue; // simple paths only
            onpath.insert(e.to);
            path.push_back(e.to);
            curCost += e.w;
            dfs(e.to);
            curCost -= e.w;
            path.pop_back();
            onpath.erase(e.to);
        }
    };
    dfs(start);
    return best;
}

// 2) DFS (plain DFS path to goal, no global visited beyond current path)
PathResult DFS_plain(const Graph& g, char start, char goal) {
    PathResult res; res.name = "DFS";
    vector<char> path; path.push_back(start);
    unordered_set<char> onpath; onpath.insert(start);
    double cost = 0.0;
    bool done = false;

    function<void(char)> dfs = [&](char u) {
        if (done) return;
        if (u == goal) {
            res.found = true; res.path = path; res.cost = cost; done = true;
            return;
        }
        vector<Edge> nbrs = g.neighbors(u);
        sort(nbrs.begin(), nbrs.end(), [](const Edge& a, const Edge& b){
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        for (const auto &e : nbrs) {
            if (onpath.count(e.to)) continue; // avoid cycles in current path
            onpath.insert(e.to);
            path.push_back(e.to);
            cost += e.w;
            dfs(e.to);
            if (done) return;
            cost -= e.w;
            path.pop_back();
            onpath.erase(e.to);
        }
    };
    dfs(start);
    return res;
}

// 3) BFS (minimum cost and optimal path) -> Uniform Cost Search (Dijkstra)
PathResult BFS_min_cost(const Graph& g, char start, char goal) {
    auto ans = ORACLE(g, start, goal);
    ans.name = "BFS (min-cost/UCS)";
    return ans;
}

// 4) IDDFS (BFS + DFS)
PathResult IDDFS(const Graph& g, char start, char goal) {
    PathResult res; res.name = "BFS+DFS (IDDFS)";
    int maxDepth = (int)g.nodes().size(); // safe bound for simple path
    for (int limit = 0; limit <= maxDepth; ++limit) {
        vector<char> path; path.push_back(start);
        unordered_set<char> onpath; onpath.insert(start);
        double cost = 0.0;
        bool found = false;

        function<void(char,int)> dls = [&](char u, int depth) {
            if (found) return;
            if (u == goal) { found = true; res.found = true; res.path = path; res.cost = cost; return; }
            if (depth == limit) return;
            vector<Edge> nbrs = g.neighbors(u);
            sort(nbrs.begin(), nbrs.end(), [](const Edge& a, const Edge& b){
                if (a.to != b.to) return a.to < b.to;
                return a.w < b.w;
            });
            for (const auto &e : nbrs) {
                if (onpath.count(e.to)) continue;
                onpath.insert(e.to);
                path.push_back(e.to);
                cost += e.w;
                dls(e.to, depth+1);
                if (found) return;
                cost -= e.w;
                path.pop_back();
                onpath.erase(e.to);
            }
        };
        dls(start, 0);
        if (found) return res;
    }
    return res;
}

// 5) DFS + History (visited set)
PathResult DFS_with_history(const Graph& g, char start, char goal) {
    PathResult res; res.name = "DFS + History";
    vector<char> path; path.push_back(start);
    unordered_set<char> visited; visited.insert(start);
    double cost = 0.0;
    bool done = false;

    function<void(char)> dfs = [&](char u) {
        if (done) return;
        if (u == goal) { res.found = true; res.path = path; res.cost = cost; done = true; return; }
        vector<Edge> nbrs = g.neighbors(u);
        sort(nbrs.begin(), nbrs.end(), [](const Edge& a, const Edge& b){
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        for (const auto &e : nbrs) {
            if (visited.count(e.to)) continue;
            visited.insert(e.to);
            path.push_back(e.to);
            cost += e.w;
            dfs(e.to);
            if (done) return;
            cost -= e.w;
            path.pop_back();
            // history keeps visited; do not unmark
        }
    };
    dfs(start);
    return res;
}

// 6) BFS + History (UCS with closed set)
PathResult BFS_with_history(const Graph& g, char start, char goal) {
    struct Node { double g; char v; long long t; };
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            if (a.g != b.g) return a.g > b.g;
            if (a.t != b.t) return a.t > b.t;
            return a.v > b.v;
        }
    };
    priority_queue<Node, vector<Node>, Cmp> pq;
    unordered_map<char,double> dist;
    unordered_map<char,char> parent;
    unordered_set<char> closed;
    TieBreaker tb;

    for (auto v : g.nodes()) dist[v] = numeric_limits<double>::infinity();
    dist[start] = 0.0;
    pq.push({0.0, start, tb.next()});

    while (!pq.empty()) {
        auto [dg,u,t] = pq.top(); pq.pop();
        if (closed.count(u)) continue;
        closed.insert(u);
        if (u == goal) {
            auto path = reconstruct(start, goal, parent);
            return {true, path, dg, "BFS + History (UCS+closed)"};
        }
        for (const auto &e : g.neighbors(u)) {
            if (closed.count(e.to)) continue;
            double nd = dg + e.w;
            if (nd < dist[e.to] - 1e-12 ||
                (fabs(nd - dist[e.to]) <= 1e-12 && (!parent.count(e.to) || u < parent[e.to]))) {
                dist[e.to] = nd;
                parent[e.to] = u;
                pq.push({nd, e.to, tb.next()});
            }
        }
    }
    return {false, {}, numeric_limits<double>::infinity(), "BFS + History (UCS+closed)"};
}

// 7) Hill Climbing (greedy by h only, no cost, no history)
PathResult HillClimbing(const Graph& g, char start, char goal) {
    PathResult res; res.name = "HC (Greedy h)";
    vector<char> path; path.push_back(start);
    unordered_set<char> onpath; onpath.insert(start);
    double totalCost = 0.0;
    char u = start;

    while (true) {
        if (u == goal) { res.found = true; res.path = path; res.cost = totalCost; return res; }
        vector<Edge> nbrs = g.neighbors(u);
        if (nbrs.empty()) break;
        sort(nbrs.begin(), nbrs.end(), [&](const Edge& a, const Edge& b){
            double ha = heuristic(a.to, goal), hb = heuristic(b.to, goal);
            if (ha != hb) return ha < hb;
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        bool moved = false;
        for (const auto &e : nbrs) {
            if (onpath.count(e.to)) continue; // avoid immediate cycles via current path
            path.push_back(e.to);
            onpath.insert(e.to);
            totalCost += e.w;
            u = e.to;
            moved = true;
            break;
        }
        if (!moved) break; // stuck (local minimum or cycle)
    }
    return res;
}

// 8) Hill Climbing + History (global visited set)
PathResult HillClimbing_with_history(const Graph& g, char start, char goal) {
    PathResult res; res.name = "HC + History";
    vector<char> path; path.push_back(start);
    unordered_set<char> visited; visited.insert(start);
    double totalCost = 0.0;
    char u = start;

    while (true) {
        if (u == goal) { res.found = true; res.path = path; res.cost = totalCost; return res; }
        vector<Edge> nbrs = g.neighbors(u);
        if (nbrs.empty()) break;
        sort(nbrs.begin(), nbrs.end(), [&](const Edge& a, const Edge& b){
            double ha = heuristic(a.to, goal), hb = heuristic(b.to, goal);
            if (ha != hb) return ha < hb;
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        bool moved = false;
        for (const auto &e : nbrs) {
            if (visited.count(e.to)) continue;
            visited.insert(e.to);
            path.push_back(e.to);
            totalCost += e.w;
            u = e.to;
            moved = true;
            break;
        }
        if (!moved) break;
    }
    return res;
}

// 9) Beam Search (width k), no history
PathResult BeamSearch(const Graph& g, char start, char goal, int beam_width) {
    PathResult res; res.name = "Beam Search";
    struct State { vector<char> path; double g; char u; long long t; };
    auto cmp = [&](const State& a, const State& b){
        double fa = heuristic(a.u, goal); // beam by h only
        double fb = heuristic(b.u, goal);
        if (fa != fb) return fa > fb;
        if (a.t != b.t) return a.t > b.t;
        return a.u > b.u;
    };
    TieBreaker tb;

    vector<State> beam;
    beam.push_back({{start}, 0.0, start, tb.next()});

    while (!beam.empty()) {
        for (const auto &s : beam) {
            if (s.u == goal) {
                res.found = true; res.path = s.path; res.cost = s.g; return res;
            }
        }
        vector<State> candidates;
        for (const auto &s : beam) {
            unordered_set<char> onp(s.path.begin(), s.path.end());
            vector<Edge> nbrs = g.neighbors(s.u);
            sort(nbrs.begin(), nbrs.end(), [&](const Edge& a, const Edge& b){
                double ha = heuristic(a.to, goal), hb = heuristic(b.to, goal);
                if (ha != hb) return ha < hb;
                if (a.to != b.to) return a.to < b.to;
                return a.w < b.w;
            });
            for (const auto &e : nbrs) {
                if (onp.count(e.to)) continue;
                auto p = s.path; p.push_back(e.to);
                candidates.push_back({move(p), s.g + e.w, e.to, tb.next()});
            }
        }
        if (candidates.empty()) break;
        sort(candidates.begin(), candidates.end(), cmp);
        if ((int)candidates.size() > beam_width) candidates.resize(beam_width);
        beam.swap(candidates);
    }
    return res;
}

// 10) Beam Search + History (global visited)
PathResult BeamSearch_with_history(const Graph& g, char start, char goal, int beam_width) {
    PathResult res; res.name = "Beam Search + History";
    struct State { vector<char> path; double g; char u; long long t; };
    auto cmp = [&](const State& a, const State& b){
        double fa = heuristic(a.u, goal);
        double fb = heuristic(b.u, goal);
        if (fa != fb) return fa > fb;
        if (a.t != b.t) return a.t > b.t;
        return a.u > b.u;
    };
    TieBreaker tb;

    vector<State> beam;
    unordered_set<char> visited;
    beam.push_back({{start}, 0.0, start, tb.next()});
    visited.insert(start);

    while (!beam.empty()) {
        for (const auto &s : beam) {
            if (s.u == goal) {
                res.found = true; res.path = s.path; res.cost = s.g; return res;
            }
        }
        vector<State> candidates;
        for (const auto &s : beam) {
            vector<Edge> nbrs = g.neighbors(s.u);
            sort(nbrs.begin(), nbrs.end(), [&](const Edge& a, const Edge& b){
                double ha = heuristic(a.to, goal), hb = heuristic(b.to, goal);
                if (ha != hb) return ha < hb;
                if (a.to != b.to) return a.to < b.to;
                return a.w < b.w;
            });
            for (const auto &e : nbrs) {
                if (visited.count(e.to)) continue;
                visited.insert(e.to);
                auto p = s.path; p.push_back(e.to);
                candidates.push_back({move(p), s.g + e.w, e.to, tb.next()});
            }
        }
        if (candidates.empty()) break;
        sort(candidates.begin(), candidates.end(), cmp);
        if ((int)candidates.size() > beam_width) candidates.resize(beam_width);
        beam.swap(candidates);
    }
    return res;
}

// 11) Branch and Bound (assume oracle bound; prune when g > bound)
PathResult BranchAndBound_with_oracle(const Graph& g, char start, char goal, double oracleCost) {
    PathResult best; best.name = "Branch and Bound (with Oracle bound)";
    best.cost = oracleCost;
    vector<char> path; path.push_back(start);
    unordered_set<char> onpath; onpath.insert(start);
    double gCost = 0.0;
    bool found = false;

    function<void(char)> dfs = [&](char u) {
        if (found) return;
        if (gCost > oracleCost + 1e-12) return;
        if (u == goal) {
            best.found = true; best.path = path; best.cost = gCost; found = true; return;
        }
        vector<Edge> nbrs = g.neighbors(u);
        sort(nbrs.begin(), nbrs.end(), [](const Edge& a, const Edge& b){
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        for (const auto &e : nbrs) {
            if (onpath.count(e.to)) continue;
            onpath.insert(e.to);
            path.push_back(e.to);
            gCost += e.w;
            dfs(e.to);
            if (found) return;
            gCost -= e.w;
            path.pop_back();
            onpath.erase(e.to);
        }
    };
    dfs(start);
    return best;
}

// 12) Cost + estimated Heuristics + Branch and Bound
// Implemented as A* with upper bound = oracle; prune when f or g exceeds bound
PathResult CostHeuristicsBranchAndBound(const Graph& g, char start, char goal, double oracleCost) {
    PathResult res; res.name = "Cost + est Heuristics + B&B";
    struct Node { double f, g; char v; long long t; };
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            if (a.f != b.f) return a.f > b.f;
            if (a.t != b.t) return a.t > b.t;
            return a.v > b.v;
        }
    };
    priority_queue<Node, vector<Node>, Cmp> open;
    unordered_map<char,double> bestg;
    unordered_map<char,char> parent;
    TieBreaker tb;

    for (auto v : g.nodes()) bestg[v] = numeric_limits<double>::infinity();
    bestg[start] = 0.0;
    open.push({heuristic(start, goal), 0.0, start, tb.next()});

    double incumbent = oracleCost; // bound

    while (!open.empty()) {
        auto cur = open.top(); open.pop();
        char u = cur.v;
        double gsofar = cur.g;
        if (gsofar > bestg[u] + 1e-12) continue;
        double f = cur.f;
        if (f > incumbent + 1e-12) continue; // prune by bound

        if (u == goal) {
            auto path = reconstruct(start, goal, parent);
            res.found = true; res.path = path; res.cost = gsofar; return res;
        }

        vector<Edge> nbrs = g.neighbors(u);
        sort(nbrs.begin(), nbrs.end(), [](const Edge& a, const Edge& b){
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        for (const auto &e : nbrs) {
            double ng = gsofar + e.w;
            if (ng > incumbent + 1e-12) continue; // branch-and-bound by oracle
            if (ng < bestg[e.to] - 1e-12 ||
                (fabs(ng - bestg[e.to]) <= 1e-12 && (!parent.count(e.to) || u < parent[e.to]))) {
                bestg[e.to] = ng;
                parent[e.to] = u;
                double nf = ng + heuristic(e.to, goal);
                open.push({nf, ng, e.to, tb.next()});
            }
        }
    }
    return res;
}

// 13) Branch and Bound Estimated Heuristics + Estimated List
// A* with open list ordered by f, closed list that avoids re-expanding worse g, and incumbent pruning
PathResult BB_EH_EL(const Graph& g, char start, char goal) {
    PathResult res; res.name = "B&B + Est Heuristics + Est List (A*+closed+incumbent)";
    struct Node { double f, g; char v; long long t; };
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            if (a.f != b.f) return a.f > b.f;
            if (a.t != b.t) return a.t > b.t;
            return a.v > b.v;
        }
    };
    priority_queue<Node, vector<Node>, Cmp> open;
    unordered_map<char,double> gbest;
    unordered_map<char,char> parent;
    unordered_set<char> closed;
    TieBreaker tb;

    for (auto v : g.nodes()) gbest[v] = numeric_limits<double>::infinity();
    gbest[start] = 0.0;
    open.push({heuristic(start, goal), 0.0, start, tb.next()});
    double incumbent = numeric_limits<double>::infinity();

    while (!open.empty()) {
        auto cur = open.top(); open.pop();
        char u = cur.v;
        double gsofar = cur.g;

        if (gsofar > gbest[u] + 1e-12) continue;
        if (closed.count(u)) continue;

        double f = cur.f;
        if (f > incumbent + 1e-12) continue; // prune by best known solution

        closed.insert(u);

        if (u == goal) {
            auto path = reconstruct(start, goal, parent);
            res.found = true; res.path = path; res.cost = gsofar;
            incumbent = min(incumbent, gsofar);
            return res;
        }

        vector<Edge> nbrs = g.neighbors(u);
        sort(nbrs.begin(), nbrs.end(), [](const Edge& a, const Edge& b){
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        for (const auto &e : nbrs) {
            double ng = gsofar + e.w;
            if (ng >= gbest[e.to] - 1e-12) continue; // avoid worse paths
            gbest[e.to] = ng;
            parent[e.to] = u;
            double nf = ng + heuristic(e.to, goal);
            if (nf > incumbent + 1e-12) continue;
            open.push({nf, ng, e.to, tb.next()});
        }
    }
    return res;
}

// 14) A* (best combination of traits)
PathResult A_star_best(const Graph& g, char start, char goal) {
    PathResult res; res.name = "A* (best)";
    struct Node { double f, g; char v; long long t; };
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            if (a.f != b.f) return a.f > b.f;
            if (a.t != b.t) return a.t > b.t;
            return a.v > b.v;
        }
    };
    priority_queue<Node, vector<Node>, Cmp> open;
    unordered_map<char,double> gbest;
    unordered_map<char,char> parent;
    unordered_map<char,double> closed_bestg; // allow reopening if better

    TieBreaker tb;
    for (auto v : g.nodes()) gbest[v] = numeric_limits<double>::infinity();
    gbest[start] = 0.0;
    open.push({heuristic(start, goal), 0.0, start, tb.next()});

    double incumbent = numeric_limits<double>::infinity();

    while (!open.empty()) {
        auto cur = open.top(); open.pop();
        char u = cur.v;
        double gsofar = cur.g;
        if (gsofar > gbest[u] + 1e-12) continue;
        double f = cur.f;
        if (f > incumbent + 1e-12) continue;

        if (u == goal) {
            auto path = reconstruct(start, goal, parent);
            res.found = true; res.path = path; res.cost = gsofar;
            incumbent = min(incumbent, gsofar);
            return res;
        }

        closed_bestg[u] = gsofar;

        vector<Edge> nbrs = g.neighbors(u);
        sort(nbrs.begin(), nbrs.end(), [](const Edge& a, const Edge& b){
            if (a.to != b.to) return a.to < b.to;
            return a.w < b.w;
        });
        for (const auto &e : nbrs) {
            double ng = gsofar + e.w;
            if (closed_bestg.count(e.to) && ng >= closed_bestg[e.to] - 1e-12) continue;
            if (ng + heuristic(e.to, goal) > incumbent + 1e-12) continue;
            if (ng < gbest[e.to] - 1e-12 ||
                (fabs(ng - gbest[e.to]) <= 1e-12 && (!parent.count(e.to) || u < parent[e.to]))) {
                gbest[e.to] = ng;
                parent[e.to] = u;
                double nf = ng + heuristic(e.to, goal);
                open.push({nf, ng, e.to, tb.next()});
            }
        }
    }
    return res;
}

// Hardcoded graph, start, goal
Graph build_graph() {
    Graph g;

    g.add_edge('A','B',1);
    g.add_edge('A','D',3);
    g.add_edge('B','C',2);
    g.add_edge('B','E',1);
    g.add_edge('C','F',4);
    g.add_edge('E','D',2);
    g.add_edge('E','F',1);
    g.add_edge('D','H',4);
    g.add_edge('H','I',2);
    g.add_edge('F','I',3);
    g.add_edge('F','G',2);
    return g;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    Graph g = build_graph();
    char start = 'A';
    char goal  = 'G';
    int beam_k = 3; // beam width

    // Oracle
    auto oracle = ORACLE(g, start, goal);

    vector<PathResult> results;

    results.push_back(BritishMuseumSearch(g, start, goal));
    results.push_back(DFS_plain(g, start, goal));
    results.push_back(BFS_min_cost(g, start, goal));
    results.push_back(IDDFS(g, start, goal));
    results.push_back(DFS_with_history(g, start, goal));
    results.push_back(BFS_with_history(g, start, goal));
    results.push_back(HillClimbing(g, start, goal));
    results.push_back(HillClimbing_with_history(g, start, goal));
    results.push_back(BeamSearch(g, start, goal, beam_k));
    results.push_back(BeamSearch_with_history(g, start, goal, beam_k));
    results.push_back(oracle); // ORACLE
    results.push_back(BranchAndBound_with_oracle(g, start, goal, oracle.cost));
    results.push_back(CostHeuristicsBranchAndBound(g, start, goal, oracle.cost));
    results.push_back(BB_EH_EL(g, start, goal));
    results.push_back(A_star_best(g, start, goal));

    for (auto &r : results) {
        cout << r.name << ":\n";
        if (r.found) {
            cout << "  Path: " << path_to_string(r.path) << "\n";
            cout << "  Cost: " << r.cost << "\n";
        } else {
            cout << "  No path found\n";
        }
    }
    return 0;
}
