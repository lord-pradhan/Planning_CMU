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
    GroundedConditionComparator>, double, setHashFn>& lookUpGIn, TreeNode* &rootIn, TreeNode* &tempPtrIn ){

    int numSym = envIn->get_symbols().size();
    auto currConditions = tempPtrIn->getCondition();

    for(auto i_action : envIn->get_actions()){

        auto actionArgCombinations = combinationCalc(envIn->get_symbols(), 
                                                            i_action.get_args().size());

        // auto argsTemp = i_action.get_args();
        // auto precondTemp = i_action.get_preconditions();

        for(auto i_args : actionArgCombinations){

            GroundedAction grdAcTemp( i_action.get_name(), i_args );
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> precondsTemp;
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effectsTemp;

            for(auto i_precond : i_action.get_preconditions()){

                auto argsAction = i_action.get_args();
                auto argsPreCond = i_precond.get_args();
                auto commonIndices = calcOverlap(argsPreCond, argsAction);
                vector<string> stringInd;
                for(auto i_temp : commonIndices){
                    stringInd.push_back(i_args[i_temp]);
                }
                GroundedCondition precondNew( i_precond.get_predicate(), stringInd );
                precondsTemp.insert(precondNew);
            }

            for(auto i_effect : i_action.get_effects()){

                auto argsAction = i_action.get_args();
                auto argsEffect = i_effect.get_args();
                auto commonIndices = calcOverlap(argsEffect, argsAction);
                vector<string> stringInd;
                for(auto i_temp : commonIndices){
                    stringInd.push_back(i_args[i_temp]);
                }
                GroundedCondition effectNew( i_effect.get_predicate(), stringInd );
                precondsTemp.insert(effectNew);
            }

            
        }
    }
}
       

    // for(auto sym1 : envIn->get_symbols()){

    //     for(auto sym2: envIn->get_symbols(); sym2!=sym1){

    //         for(auto action : envIn->get_actions()){

    //             GroundedAction actionTemp(action.get_name(), {sym1, sym2});

    //             unordered_set<Condition, ConditionHasher, ConditionComparator> precondTemp;
    //             for(auto cond : action.get_preconditions()){

    //                 // if(cond.get_args().size()==1)
    //                 // precondTemp.push_back(cond.get_predicate(),{sym1, sym2})
    //             }
    //         }
    //     }
    // }


vector<int> calcOverlap(list<string> smaller, list<string> larger){ 
    
    vector<int> indices;

    for(auto i: smaller){

        auto it = find(larger.begin(), larger.end(), i);
        if(it!=larger.end()){

            indices.push_back(it - larger.begin());
        }
    }
    return indices;
} 

vector< vector<string> > combinationCalc( const unordered_set<string>& 
                    symbolsIn, int K ){

    vector< vector<string> > actionArgCombinations;
    int N = symbolsIn.size();
    string bitmask(K, 1); // K leading 1's
    bitmask.resize(N, 0); // N-K trailing 0's

    // print integers and permute bitmask
    int combnCt =0;
    do {

        int symCt=0;
        vector<string> listTemp;
        for (auto i_sym : symbolsIn){}{

            if (bitmask[symCt]){

                listTemp.push_back(i_sym);
                cout<< "symbol inserted is "<<i_sym<<endl;
            }

            symCt++;
        }

        actionArgCombinations.push_back(listTemp);
        combnCt++;

    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

    return actionArgCombinations;
}


void comb(int N, int K)
{
    std::string bitmask(K, 1); // K leading 1's
    bitmask.resize(N, 0); // N-K trailing 0's

    // print integers and permute bitmask
    do {
        for (int i = 0; i < N; ++i) // [0..N-1] integers
        {
            if (bitmask[i]) std::cout << " " << i;
        }
        std::cout << std::endl;
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
}