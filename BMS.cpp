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
