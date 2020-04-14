#include "utilfunction.h"
#include <limits>


// Tree Node class //
TreeNode::TreeNode(){

    G_val = numeric_limits<double>::infinity();
    expanded = false;
    parent = nullptr;
}

unordered_set<GroundedCondition, GroundedConditionHasher, 
    GroundedConditionComparator> TreeNode::getCondition() const{

    return conditions;       
}

vector<TreeNode*> TreeNode::getSuccesors() const{

    return succesors;
}

TreeNode* TreeNode::getParent() const{

    return parent;
}

double TreeNode::getG() const{

    return G_val;
}

bool TreeNode::isExpanded() const{

    return expanded;
}

GroundedAction TreeNode::getNextAction() const{

    return nextAction;
}

void TreeNode::setCondition(unordered_set<GroundedCondition, 
        GroundedConditionHasher, 
        GroundedConditionComparator> conditionsIn){

    conditions = conditionsIn;
}

void TreeNode::addSuccesor(TreeNode* succIn){

    succesors.push_back(succIn);
}

void TreeNode::setSuccesors(const vector<TreeNode*>& succIns){

    succesors = succIns;
}

void TreeNode::setParent(TreeNode* parentIn){

    parent = parentIn; 
}

void TreeNode::setG(const double& G_in){

    G_val = G_in;
}

void TreeNode::expand(){expanded = true;}

void TreeNode::setNextAction(GroundedAction actionIn){

    nextAction = actionIn;
}

void TreeNode::calcSuccesors(Env* envIn, vector<GroundedAction>& nextActionsIn, 
    vector< unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> >&
    succesorConditionsIn, unordered_map< unordered_set<GroundedCondition, GroundedConditionHasher, 
    GroundedConditionComparator>, double, setHashFn>& lookUpGIn, TreeNode* &rootIn ){

    int numSym = envIn->symbols.size();
    
    for(auto i1 : envIn->symbols){

        for(auto i2: envIn->symbols; i2!=i1){

            // Action actionTemp()
        }
    }
}