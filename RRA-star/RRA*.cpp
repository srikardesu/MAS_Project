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

auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
std::mt19937 mt(seed);

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
        for (auto c : dir) {
            int x = curr.ss.fs + c.fs;
            int y = curr.ss.ss + c.ss;
            pi curxy = {x, y};
            if (check(x, y) and closed.find({x, y}) == closed.end()) {
                int newG = G[curr.ss.fs][curr.ss.ss] + MH;
                int newH = manhattan(start, {x, y});

                int oldF = F[x][y];
                if (newG + newH < F[x][y]) {
                    F[x][y] = newG + newH;
                    G[x][y] = newG;
                    if (open.find({oldF, {x, y}}) != open.end()) {
                        open.erase(open.find({oldF, {x, y}}));
                    }
                    open.insert({{F[x][y], {x, y}}});
                }
            }
        }
        if (curr.ss == node) return true;
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

void print_grid() {
    auto newgrid = grid;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if (closed.find({i, j}) != closed.end()) {
                newgrid[i][j] = 'D';
            }
        }
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

vector<vector<char>> randgrid(int per) {
    vector<vector<char>> ngrid(n, vector<char>(m, '.'));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if ((i == 0 and j == 0) or (i == n - 1 and j == m - 1)) continue;
            (mt() % 100 < per) ? ngrid[i][j] = 'X' : ngrid[i][j] = '.';
        }
    }
    return ngrid;
}

int main() {
    // take input of n*m size grid
    cin >> n >> m;
    grid = vector<vector<char>>(n, vector<char>(m, '.'));

    // store start positions
    vector<pair<int, int>> starts, goals;
    int type;
    cin >> type;
    if (type) {
        int val;
        cin >> val;
        grid = randgrid(val);
    } else {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                cin >> grid[i][j];
            }
        }
    }
    start = {0, 0};
    pi goal = {n - 1, m - 1};
    print_grid();
    G.assign(n, vector<int>(m, inf));
    F.assign(n, vector<int>(m, inf));
    initialise_rra(start, goal);
    // num inputs of node
    int c;
    cin >> c;
    while (c--) {
        pi node;
        cin >> node.fs >> node.ss;
        cout << abstractDistance(node) << endl;
        print_grid();
    }

    return 0;
}