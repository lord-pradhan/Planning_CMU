#include <vector>
#include <string>
#include <list>
#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <limits>

using namespace std;

#ifndef UTILFUNCTION_H
#define UTILFUNCTION_H

// class GroundedCondition;
// class Condition;
// class GroundedAction;
// class Action;
// class Env;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true);

    GroundedCondition(const GroundedCondition& gc);

    string get_predicate() const;

    list<string> get_arg_values() const;

    bool get_truth() const;

    void set_truth(bool truthIn);

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred){

		os << pred.toString() << " ";
		return os;
    }

    bool operator==(const GroundedCondition& rhs) const;

    string toString() const;
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        // cout<<"Hash value is "<<hash<string>{}(gcond.toString());
        return hash<string>{}(gcond.toString());
    }
};


class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth);

    string get_predicate() const;

    list<string> get_args() const;

    bool get_truth() const;

    friend ostream& operator<<(ostream& os, const Condition& cond){

    	os << cond.toString() << " ";
	    return os;
    }

    bool operator==(const Condition& rhs) const; // fixed

    string toString() const;
};


struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};



class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects);

    string get_name() const;

    list<string> get_args() const;

    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const;

    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const;

    bool operator==(const Action& rhs) const;

    friend ostream& operator<<(ostream& os, const Action& ac){

	    os << ac.toString() << endl;
	    os << "Precondition: ";
	    for (Condition precond : ac.get_preconditions())
	        os << precond;
	    os << endl;
	    os << "Effect: ";
	    for (Condition effect : ac.get_effects())
	        os << effect;
	    os << endl;
	    return os;
    }

    string toString() const;
};


struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};



class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPreconditions;
    // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffects;

public:
    GroundedAction(string name, list<string> arg_values);

    string get_name() const;

    list<string> get_arg_values() const;

    bool operator==(const GroundedAction& rhs) const;

    friend ostream& operator<<(ostream& os, const GroundedAction& gac){

	    os << gac.toString() << " ";
	    return os;
    }

    string toString() const;
};



class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc);

    void add_initial_condition(GroundedCondition gc);

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_condition() const;

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_condition() const;

    void add_goal_condition(GroundedCondition gc);

    void remove_goal_condition(GroundedCondition gc);

    void add_symbol(string symbol);

    void add_symbols(list<string> symbols);

    void add_action(Action action);

    unordered_set<Action, ActionHasher, ActionComparator> get_actions();

    Action get_action(string name);

    unordered_set<string> get_symbols() const;

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};



class TreeNode{

private:
	unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> conditions;
	double G_val;
	vector<TreeNode*> succesors;
	vector<GroundedAction> nextActions;
	TreeNode* parent;
	bool expanded;
	

public:
	TreeNode();

	unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getCondition() const;
	vector<TreeNode*> getSuccesors() const;
	TreeNode* getParent() const;
	double getG() const;
	bool isExpanded() const;
	vector<GroundedAction> getNextAction() const;

	void setCondition(
		unordered_set<GroundedCondition, 
		GroundedConditionHasher, 
		GroundedConditionComparator> conditionsIn);

	void addSuccesor(TreeNode* succIn);
	void addNextAction(GroundedAction actionIn);
	void setSuccesors(const vector<TreeNode*>& succIns);	
	void setParent(TreeNode* parentIn);
	void setG(const double& G_in);
	void expand();
	void setNextActions(vector<GroundedAction> actionsIn);
};


struct CompareF{
    bool operator()(TreeNode* t1, TreeNode* t2) {
        // return "true" if "p1" is ordered before "p2", for example:
        long eps = 1;
        return t1->getG() > t2->getG();
    }
};
// class setHashFn{ 
// public: 
  
//     // Use sum of lengths of first and last names 
//     // as hash function. 
//     double operator()(const unordered_set<GroundedCondition, GroundedConditionHasher, 
//     					GroundedConditionComparator>& setIn) const{ 

//     	// int sizetemp = setIn.size(); 
//     	double sumHash = 0;

//     	for(auto i : setIn){

//         	sumHash += hash<string>{}(i.toString());
//         }

//         return sumHash;
// 	}
// } 
bool distinctElems(list<string> candidate);

vector< list<string> > combinationCalc( const unordered_set<string>& symbolsIn, int K );

list<string> calcOverlap(list<string> smaller, list<string> larger, list<string> argsIn,
	unordered_set<string> symbolsIn);

bool precondCheck(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>
    &precondsTempIn, const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>&
    currConditionsIn );

void calcSuccesors(Env* envIn, priority_queue< TreeNode*, vector<TreeNode*>, CompareF >& open_setIn , 
    TreeNode* &tempPtrIn );

#endif