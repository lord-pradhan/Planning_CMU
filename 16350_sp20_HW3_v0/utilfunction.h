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

using namespace std;

#ifndef UTILFUNCTION_H
#define UTILFUNCTION_H

// class GroundedCondition;
// class Condition;
// class GroundedAction;
// class Action;
// class Env;

// Env* create_env(char* filename);

// list<string> parse_symbols(string symbols_str);

struct CompareF{
    bool operator()(TreeNode* t1, TreeNode* t2) {
        // return "true" if "p1" is ordered before "p2", for example:
        long eps = 1;
        return t1->getG() > t2->getG();
    }
};

class TreeNode{

private:
	unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> conditions;
	double G_val;
	vector<TreeNode*> succesors;
	TreeNode* parent;
	bool expanded;
	GroundedAction nextAction;

public:
	TreeNode();

	unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getCondition() const;
	vector<TreeNode*> getSuccesors() const;
	TreeNode* getParent() const;
	double getG() const;
	bool isExpanded() const;
	GroundedAction getNextAction() const;

	void setCondition(
		unordered_set<GroundedCondition, 
		GroundedConditionHasher, 
		GroundedConditionComparator> conditionsIn);

	void addSuccesor(TreeNode* succIn);

	vector<GroundedAction> calcSuccesors(const Env* envIn);
	void setSuccesors(const vector<TreeNode*>& succIns);	
	void setParent(TreeNode* parentIn);
	void setG(const double& G_in);
	void expand();
	void setNextAction(GroundedAction actionIn);
};

class setHashFn{ 
public: 
  
    // Use sum of lengths of first and last names 
    // as hash function. 
    double operator()(const unordered_set<GroundedCondition, GroundedConditionHasher, 
    					GroundedConditionComparator>& setIn) const{ 

    	// int sizetemp = setIn.size(); 
    	double sumHash = 0;

    	for(auto i : setIn){

        	sumHash += hash<string>{}(i.toString());
        }

        return sumHash;
	}
} 

#endif