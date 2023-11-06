#include <bits/stdc++.h>

using namespace std;
#define pi pair<int, int>
#define fs first
#define ss second
#define vi vector<int>
#define vpi vector<pi>
#define vvpi vector<vpi>
#define pb push_back

int n, m;
vector<vector<char>> grid;
int inf = 1e9;

int MH = 10;
int manhattan(pair<int, int> a, pair<int, int> b) {
    return (abs(a.first - b.first) + abs(b.second - a.second)) * MH;
}

int f(int curr, int cost, pi node, pi end) {
    return curr + cost + manhattan(node, end);
}

bool check(int x, int y) {
    if (x < n and y < m and x >= 0 and y >= 0 and grid[x][y] != 'X') return true;
    return false;
}

vpi dir = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

// start node, end node, block flag, blocked node (explicitly for replanner)

pair<bool, vector<pi>> astar(pi start, pi goal, bool blockFlag, vpi& block) {
    set<pair<vi, pi>> nodes;
    if (blockFlag) {
        for (auto bl : block)
            grid[bl.fs][bl.ss] = 'X';
    }
    // {{fcost, gcost, hcost}, node}
    nodes.insert({{f(0, 0, start, goal), 0, f(0, 0, start, goal)}, start});
    vector<vector<int>> G(n, vector<int>(m, inf));
    vector<vector<int>> F(n, vector<int>(m, inf));
    vector<vector<pi>> par(n, vector<pi>(m, {inf, inf}));
    par[start.fs][start.ss] = {-1, -1};
    while (nodes.size()) {
        auto beg = *nodes.begin();
        // current node
        auto from = beg.ss;
        // cost vector of curr node
        vi cost = beg.fs;
        if (cost[0] > F[from.fs][from.ss]) continue;
        // F cost
        F[from.fs][from.ss] = cost[0];
        // G cost
        G[from.fs][from.ss] = cost[1];
        nodes.erase(nodes.begin());
        int _f = F[from.fs][from.ss];
        int _g = G[from.fs][from.ss];
        for (auto c : dir) {
            int x = from.fs + c.fs, y = from.ss + c.ss;
            if (!check(x, y)) continue;
            int heref = f(_g, MH, {x, y}, goal);
            if (heref < F[x][y]) {
                par[x][y] = {from.fs, from.ss};
                F[x][y] = heref;
                G[x][y] = _g + MH;
                nodes.insert({{F[x][y], G[x][y], F[x][y] - G[x][y]}, {x, y}});
            }
        }
    }
    if (blockFlag) {
        for (auto bl : block)
            grid[bl.fs][bl.ss] = '.';
    }
    if (F[goal.fs][goal.ss] >= inf) {
        return {false, {}};
    }
    vpi path;
    int x = goal.fs, y = goal.ss;
    while (x != -1) {
        path.emplace_back(x, y);
        int _x = par[x][y].fs;
        int _y = par[x][y].ss;
        x = _x, y = _y;
    }
    reverse(path.begin(), path.end());
    return {true, path};
}

void print_grid(vpi& pos) {
    auto newgrid = grid;
    for (int i = 0; i < (int)pos.size(); i++) {
        newgrid[pos[i].fs][pos[i].ss] = 'A' + i;
    }
    cout << "----------------\n";
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            cout << newgrid[i][j] << " ";
        }
        cout << "\n";
    }
    cout << "----------------\n";
    cout << "\n";
}

int main() {
    // take input of n*m size grid
    cin >> n >> m;
    grid = vector<vector<char>>(n, vector<char>(m, '.'));

    // store start positions
    vector<pair<int, int>> starts, goals;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            cin >> grid[i][j];
        }
    }
    int num_agents;
    cin >> num_agents;
    // enter start position and respective goal
    // m start goal pairs
    for (int i = 0; i < num_agents; i++) {
        pair<int, int> start, goal;
        cin >> start.first >> start.second;
        cin >> goal.first >> goal.second;
        starts.push_back(start);
        goals.push_back(goal);
    }
    vvpi Agent_Paths;
    for (int i = 0; i < num_agents; i++) {
        // ith agent
        vpi pass;
        pair<bool, vpi> got = astar(starts[i], goals[i], 0, pass);
        if (got.fs == false) {
            cout << "There is no available path for agent: " << i + 1 << "\n";
            return 0;
        }
        Agent_Paths.pb(got.ss);
    }
    for (int i = 0; i < (int)Agent_Paths.size(); i++) {
        cout << "Path for agent " << i + 1 << ":\n";
        for (auto c : Agent_Paths[i]) {
            cout << "(" << c.fs << ", " << c.ss << "), ";
        }
        cout << "\n";
    }

    vector<int> vis(num_agents);
    vector<int> agi(num_agents);
    vector<int> ptr(num_agents);
    vpi pos(num_agents);
    for (int i = 0; i < num_agents; i++) {
        pos[i] = starts[i];
    }
    // check if some agent reached end goal
    for (int i = 0; i < num_agents; i++) {
        if (pos[i] == goals[i]) {
            vis[i] = 1;
        }
    }
    vi prevstate(num_agents);
    prevstate = vi(num_agents, -1);
    while (1) {
        vi currstate = ptr;
        if (currstate == prevstate) {
            cout << "All agents blocked, no path found for any. EXITING\n";
            return 0;
        }
        map<pi, int> agentpres;
        for (int i = 0; i < num_agents; i++) {
            agentpres[pos[i]] = 1;
        }
        print_grid(pos);
        bool f = true;
        for (int i = 0; i < num_agents; i++) f = f & vis[i];
        if (f) {
            cout << "simulation for all agents done\n";
            return 0;
        }
        // each next node store who all will go into it
        map<pi, set<pi>> mp;
        for (int i = 0; i < num_agents; i++) {
            // iterate over all agents
            if (!vis[i]) {
                mp[Agent_Paths[i][ptr[i] + 1]].insert({-agi[i], i});
            }
        }
        // highest agitation guy process first
        for (auto collision_nodes : mp) {
            pi node = collision_nodes.fs;
            auto st = collision_nodes.ss;
            // if ((int)st.size() == 1LL) {
            //     auto beg = *st.begin();
            //     pos[beg.ss] = node;
            //     // reached goal
            //     if (pos[beg.ss] == goals[beg.ss]) {
            //         vis[beg.ss] = 1;
            //     }
            //     ptr[beg.ss]++;
            // } else {
            //     // guy with highest agi gets
            //     // rest all replan
            //     auto beg = *st.begin();
            //     pos[beg.ss] = node;
            //     // reached goal
            //     if (pos[beg.ss] == goals[beg.ss]) {
            //         vis[beg.ss] = 1;
            //     }
            //     ptr[beg.ss]++;
            //     st.erase(st.begin());
            // replan
            for (auto c : st) {
                int curr = c.ss;
                // cout << curr << "\n";
                vpi block;
                for (auto c : dir) {
                    int x = pos[curr].fs + c.fs, y = pos[curr].ss + c.ss;
                    if (check(x, y) and agentpres[{x, y}]) block.pb({x, y});
                }
                pair<bool, vpi> np = astar(pos[curr], goals[curr], 1, block);
                vpi newpath = np.ss;
                if (!np.fs) {
                    cout << "agent: " << curr << " waits as it couldnt find an optimal path.\n";
                    agi[curr]++;
                    continue;
                }
                while (Agent_Paths[curr].back().fs != pos[curr].fs or Agent_Paths[curr].back().ss != pos[curr].ss) {
                    Agent_Paths[curr].pop_back();
                }
                Agent_Paths[curr].pop_back();
                // add the new path
                for (auto c : newpath) Agent_Paths[curr].pb(c);
                // update ptr ig
                ptr[curr]++;
                // update new pos
                agentpres[pos[curr]]--;
                pos[curr] = Agent_Paths[curr][ptr[curr]];
                agentpres[pos[curr]]++;
                // update agitation of collision nodes
                agi[curr]++;
            }
        }

        // check if some agent reached end goal
        for (int i = 0; i < num_agents; i++) {
            if (pos[i] == goals[i]) {
                vis[i] = 1;
            }
        }
        prevstate = currstate;
    }
    return 0;
}