////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Sushant Chavan

#include <smpl/search/arastar_egraph.h>
#include <smpl/extension.h>

// #include <algorithm>

// // system includes
// #include <sbpl/utils/key.h>

// // project includes
// #include <smpl/time.h>
#include <smpl/console/console.h>

namespace smpl {

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";

ARAStarEGraph::ARAStarEGraph(
    RobotPlanningSpace* space,
    RobotHeuristic* heur)
: ARAStar(space, heur)
{
    m_ege = space->getExtension<ExperienceGraphExtension>();
    if (!m_ege) {
        SMPL_WARN_ONCE("ExperienceGraphPlanner recommends ExperienceGraphExtension");
    }

    m_egh = heur->getExtension<ExperienceGraphHeuristicExtension>();
    if (!m_egh) {
        SMPL_WARN_ONCE("ExperienceGraphPlanner recommends ExperienceGraphHeuristic");
    }
}

ARAStarEGraph::~ARAStarEGraph()
{
    // for (SearchState* s : m_states) {
    //     if (s != NULL) {
    //         delete s;
    //     }
    // }
}

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and INCONS list appropriately.
void ARAStarEGraph::expand(SearchState* s)
{
    m_succs.clear();
    m_costs.clear();
    m_space->GetSuccs(s->state_id, &m_succs, &m_costs);

    SMPL_DEBUG_NAMED(SELOG, "  %zu successors", m_succs.size());

    for (size_t sidx = 0; sidx < m_succs.size(); ++sidx) {
        int succ_state_id = m_succs[sidx];
        int cost = m_costs[sidx];

        SearchState* succ_state = getSearchState(succ_state_id);
        reinitSearchState(succ_state);

        int new_cost = s->eg + cost;
        SMPL_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
        if (new_cost < succ_state->g) {
            succ_state->g = new_cost;
            succ_state->bp = s;
            if (succ_state->iteration_closed != m_iteration) {
                succ_state->f = computeKey(succ_state);
                if (m_open.contains(succ_state)) {
                    m_open.decrease(succ_state);
                } else {
                    m_open.push(succ_state);
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }
        }
    }

    if (m_ege) {
        std::vector<int> snap_succs;
        m_egh->getEquivalentStates(s->state_id, snap_succs);

        for (size_t sidx = 0; sidx < snap_succs.size(); ++sidx) {
            int snap_id = snap_succs[sidx];
            int cost;
            if (!m_ege->snap(s->state_id, snap_id, cost)) {
                continue;
            }

            SearchState* snap_state = getSearchState(snap_id);
            reinitSearchState(snap_state);

            if (snap_state->iteration_closed != 0) {
                continue;
            }

            int new_cost = s->eg + cost;
            if (new_cost < snap_state->g) {
                snap_state->g = new_cost;
                snap_state->bp = s;
                if (snap_state->iteration_closed != m_iteration) {
                    snap_state->f = computeKey(snap_state);
                    if (m_open.contains(snap_state)) {
                        m_open.decrease(snap_state);
                    } else {
                        m_open.push(snap_state);
                    }
                } else if (!snap_state->incons) {
                    m_incons.push_back(snap_state);
                }
            }
        }

        std::vector<int> shortcut_succs;
        m_egh->getShortcutSuccs(s->state_id, shortcut_succs);

        for (size_t sidx = 0; sidx < shortcut_succs.size(); ++sidx) {
            int scut_id = shortcut_succs[sidx];
            int cost;
            if (!m_ege->shortcut(s->state_id, scut_id, cost)) {
                continue;
            }

            SearchState* scut_state = getSearchState(scut_id);
            reinitSearchState(scut_state);

            if (scut_state->iteration_closed != 0) {
                continue;
            }

            int new_cost = s->eg + cost;
            if (new_cost < scut_state->g) {
                scut_state->g = new_cost;
                scut_state->bp = s;
                if (scut_state->iteration_closed != m_iteration) {
                    scut_state->f = computeKey(scut_state);
                    if (m_open.contains(scut_state)) {
                        m_open.decrease(scut_state);
                    } else {
                        m_open.push(scut_state);
                    }
                } else if (!scut_state->incons) {
                    m_incons.push_back(scut_state);
                }
            }
        }
    }
}

} // namespace smpl
