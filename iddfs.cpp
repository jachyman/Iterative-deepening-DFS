#include <queue>
#include <unordered_map>

#include "iddfs.h"

// Naimplementujte efektivni algoritmus pro nalezeni nejkratsi (respektive nej-
// levnejsi) cesty v grafu. V teto metode mate ze ukol naimplementovat pametove
// efektivni algoritmus pro prohledavani velkeho stavoveho prostoru. Pocitejte
// s tim, ze Vami navrzeny algoritmus muze bezet na stroji s omezenym mnozstvim
// pameti (radove nizke stovky megabytu). Vhodnym pristupem tak muze byt napr.
// iterative-deepening depth-first search.
//
// Metoda ma za ukol vratit ukazatel na cilovy stav, ktery je dosazitelny pomoci
// nejkratsi/nejlevnejsi cesty.

using ull = unsigned long long;
std::unordered_map<ull, int> visited;

int bfs_depth = 2;

state_ptr best_state;
int best_depth;

std::pair<state_ptr, int> serial_bfs(state_ptr root, int root_depth) {
    std::queue<std::pair<state_ptr, int>> queue;

    queue.emplace(root, 0);
    state_ptr bfs_best_state;
    int bfs_best_depth;
    bool goal_found = false;
    while (!queue.empty()) {
        state_ptr state = queue.front().first;
        int depth = queue.front().second;
        queue.pop();

        if (goal_found && depth > bfs_best_depth) {
            break;
        }

        if (state->is_goal() && (bfs_best_state == nullptr || state->get_identifier() < bfs_best_state->get_identifier())) {
            bfs_best_state = state;
            bfs_best_depth = depth;
            goal_found = true;
            continue;
        }
        if (depth > bfs_depth) {
            continue;
        }
        for (state_ptr next_state : state->next_states()) {
            queue.emplace(next_state, depth + 1);
        }
    }

    return std::make_pair(bfs_best_state, bfs_best_depth + root_depth);
}

std::pair<state_ptr, int> depth_limited_search(state_ptr root, int depth, int max_depth) {

    if (depth == max_depth - bfs_depth) {
        return serial_bfs(root, depth);
    }
    /*
    if (depth == max_depth) {
        return std::make_pair(nullptr,0);
    }
    if (root->is_goal()) {
        return std::make_pair(root, depth);
    }
     */

    for (state_ptr next_state: root->next_states()) {
#pragma omp task
        {
            ull next_state_id = next_state->get_identifier();
            int next_state_depth = depth + 1;
            bool state_was_visited = true;
#pragma omp critical
            if (visited.find(next_state_id) == visited.end() ||  next_state_depth < visited[next_state_id]) {
                state_was_visited = false;
            }
            if (!state_was_visited) {
#pragma omp critical
                visited[next_state_id] = next_state_depth;

                std::pair<state_ptr, int> res = depth_limited_search(next_state, depth + 1, max_depth);
                state_ptr res_state = res.first;
                int res_depth = res.second;

                if (res_state != nullptr) {
#pragma omp critical
                    if (res_depth < best_depth ||
                        (res_depth == best_depth && res_state->get_identifier() < best_state->get_identifier())) {
                        best_state = res_state;
                        best_depth = res_depth;
                    }
                }
            }
        }
    }

    return std::make_pair(best_state, best_depth);
}

state_ptr iddfs(state_ptr root) {

    int max_depth = bfs_depth;
    best_depth = INT_MAX;

    while (true) {
        //printf("MAX DEPTH %d\n", max_depth);
        std::pair<state_ptr, int> res;
        visited.clear();
#pragma omp parallel
        {
#pragma omp single
            res = depth_limited_search(root, 0, max_depth);
        }
        state_ptr result = res.first;
        if (result != nullptr) {
            return result;
        }
        max_depth++;
    }

    return root;
}