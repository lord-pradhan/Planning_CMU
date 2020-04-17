#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include "utilfunction.h"
#include <queue>
#include <chrono> 
using namespace std::chrono;

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

using namespace std;

bool print_status = true;




list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}



///////////////// ****** Planner *******////////////
list<GroundedAction> planner(Env* env)
{
    // this is where you insert your planner
    auto start = high_resolution_clock::now();    

    // initialize the root of tree
    TreeNode* root = new TreeNode;
    root->setParent(nullptr);
    root->setG(0.0);
    root->calcH(env->get_goal_condition());
    root->setCondition(env->get_initial_condition());
    // lookUpG[env->get_initial_condition()] = 0.0;

    priority_queue< TreeNode*, vector<TreeNode*>, CompareF > open_set;
    open_set.push(root);
    // root->expand();

    while( (!open_set.empty()) &&
     !( precondCheck( env->get_goal_condition(), open_set.top()->getCondition() ) ) ){

        TreeNode* tempPtr = open_set.top();
        tempPtr->expand();
        open_set.pop();

        // vector<GroundedAction> nextActions;
        // vector< unordered_set<GroundedCondition, GroundedConditionHasher, 
        //         GroundedConditionComparator> > succesorConditions;
        
        // expand tree inside function
        calcSuccesors(env, open_set, tempPtr);        
    }

    // // stack<TreeNode*> optPath;
    stack<GroundedAction> actionQueue;
    list<GroundedAction> actions;

    if( !precondCheck( env->get_goal_condition(),  open_set.top()->getCondition() ) ){
        //open_set.top()->getCondition() !=env->get_goal_condition()){

        cout<<"Open set empty \n";
    }
    else{
        
        cout<<"target expanded! \n";
        TreeNode* traverse = open_set.top();

        while(traverse->getParent()!=nullptr){

            auto condTemp = traverse->getCondition();

            // cout<<"condTemp is ";
            // for(auto i : condTemp){cout<<i<<", ";}
            // cout<<endl;

            traverse = traverse->getParent();
            int i_ct=0;
            for(auto i : traverse->getSuccesors()){

                // cout<<"i->getCondition is ";
                // for(auto j : i->getCondition()){cout<<j<<", ";}
                // cout<<endl;

                if(i->getCondition() == condTemp){

                    actionQueue.push(traverse->getNextAction()[i_ct]);
                    cout<<"broke"<<endl;
                    break;
                }

                i_ct++;
            }
        }
    }

    while(!actionQueue.empty()){

        actions.push_back(actionQueue.top());
        actionQueue.pop();
    }

    // for(auto i : env->get_actions() ){

    //     cout<<"Action is "<<i<<endl;
    //     // for(auto j:i.get_args()){
    //     //     cout<<"Args are "<<j<<endl;
    //     // }
    // }

    // // blocks world example
    
    // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop-start);
    cout<<"Planner runs in time "<<duration.count()<<" microseconds"<<endl;
    return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}