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

set<pair<int, pi>> open;
set<pi> closed;
vector<vector<int>> G, F;
pi start;

void initialise_rra(pi start, pi goal) {
    G[goal.fs][goal.ss] = 0;
    F[goal.fs][goal.ss] = manhattan(goal, start);
    open.insert({F[goal.fs][goal.ss], goal});
    return;
}

bool rra(pi node) {
    // perform rra* till you reach node
    while (!open.empty()) {
        pair<int, pi> curr = *open.begin();
        open.erase(open.begin());
        closed.insert(curr.ss);
        if (curr.ss == node) return true;
        for (auto c : dir) {
            int x = curr.ss.fs + c.fs;
            int y = curr.ss.ss + c.ss;
            if (!check(x, y)) continue;
            int oldF = F[x][y];
            G[x][y] = G[curr.ss.fs][curr.ss.ss] + MH;
            F[x][y] = manhattan(start, {x, y}) + G[x][y];
            if (open.find({oldF, {x, y}}) == open.end() and closed.find({x, y}) == closed.end()) {
                open.insert({{F[x][y], {x, y}}});
            }
            if (open.find({oldF, {x, y}}) != open.end() and F[x][y] < oldF) {
                open.erase(open.find({oldF, {x, y}}));
                open.insert({F[x][y], {x, y}});
            }
        }
    }
    return false;
}

int abstractDistance(pi node) {
    if (closed.find(node) != closed.end()) {
        return G[node.fs][node.ss];
    }
    if (rra(node)) {
        return G[node.fs][node.ss];
    }
    return inf;
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
    start = {-1, -1};
    pi goal = {-1, -1};
    // take input of start and goal
    cin >> start.fs >> start.ss;
    cin >> goal.fs >> goal.ss;
    G.assign(n, vector<int>(m, inf));
    F.assign(n, vector<int>(m, inf));
    initialise_rra(start, goal);

    return 0;
}