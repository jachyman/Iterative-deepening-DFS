#include <queue>
#include <unordered_map>
#include <shared_mutex>
#include <mutex>

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
std::unordered_map<ull, unsigned int> visited_weighted;

unsigned int bfs_cost = 10;

state_ptr best_state_weighted;
unsigned int best_cost;

//std::shared_mutex mutex;

std::pair<state_ptr, int> serial_bfs(state_ptr root, unsigned int root_cost) {
    std::queue<std::pair<state_ptr, int>> queue;

    queue.emplace(root, 0);
    state_ptr bfs_best_state;
    unsigned int bfs_best_cost;
    bool goal_found = false;
    while (!queue.empty()) {
        state_ptr state = queue.front().first;
        unsigned int cost = queue.front().second;
        queue.pop();

        if (goal_found && cost > bfs_best_cost) {
            break;
        }

        if (state->is_goal() && (bfs_best_state == nullptr || state->get_identifier() < bfs_best_state->get_identifier())) {
            bfs_best_state = state;
            bfs_best_cost = cost;
            goal_found = true;
            continue;
        }
        if (cost > bfs_cost) {
            continue;
        }
        for (state_ptr next_state : state->next_states()) {
            queue.emplace(next_state, state->current_cost());
        }
    }

    return std::make_pair(bfs_best_state, bfs_best_cost + root_cost);
}

std::pair<state_ptr, int> depth_limited_search(state_ptr root, unsigned int cost, unsigned int max_cost) {

    if (cost > max_cost) {
        return std::make_pair(nullptr, 0);
    }
    if (cost >= max_cost - bfs_cost) {
        return serial_bfs(root, cost);
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
            bool state_was_visited = true;
#pragma omp critical
            {
                //std::shared_lock<std::shared_mutex> lock(mutex);
                if (visited_weighted.find(next_state_id) == visited_weighted.end() ||  next_state->current_cost() < visited_weighted[next_state_id]) {
                    state_was_visited = false;
                }
            }
            if (!state_was_visited) {
#pragma omp critical
                {
                    //std::unique_lock<std::shared_mutex> lock(mutex);
                    visited_weighted[next_state_id] = next_state->current_cost();
                }

                std::pair<state_ptr, int> res = depth_limited_search(next_state, next_state->current_cost(), max_cost);
                state_ptr res_state = res.first;
                unsigned int res_cost = res.second;

                if (res_state != nullptr) {
#pragma omp critical
                    if (res_cost < best_cost ||
                        (res_cost == best_cost && res_state->get_identifier() < best_state_weighted->get_identifier())) {
                        best_state_weighted = res_state;
                        best_cost = res_cost;
                    }
                }
            }
        }
    }

    return std::make_pair(best_state_weighted, best_cost);
}

state_ptr iddfs_weighted(state_ptr root) {

    unsigned int max_cost = bfs_cost;
    best_cost = INT_MAX;

    while (true) {
        //printf("MAX DEPTH %d\n", max_depth);
        std::pair<state_ptr, int> res;
        visited_weighted.clear();
#pragma omp parallel
        {
#pragma omp single
            res = depth_limited_search(root, 0, max_cost);
        }
        state_ptr result = res.first;
        if (result != nullptr) {
            return result;
        }
        max_cost++;
    }

    return nullptr;
}