#include "utilfunction.h"


using namespace std;


//Predefined classes
GroundedCondition::GroundedCondition(string predicate, list<string> arg_values, bool truth)
{
    this->predicate = predicate;
    this->truth = truth;  // fixed
    for (string l : arg_values)
    {
        this->arg_values.push_back(l);
    }
}

GroundedCondition::GroundedCondition(const GroundedCondition& gc)
{
    this->predicate = gc.predicate;
    this->truth = gc.truth;  // fixed
    for (string l : gc.arg_values)
    {
        this->arg_values.push_back(l);
    }
}

string GroundedCondition::get_predicate() const
{
    return this->predicate;
}
list<string> GroundedCondition::get_arg_values() const
{
    return this->arg_values;
}

bool GroundedCondition::get_truth() const
{
    return this->truth;
}

void GroundedCondition::set_truth(bool truthIn){

    truth = truthIn;
}


bool GroundedCondition::operator==(const GroundedCondition& rhs) const
{
    if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
        return false;

    auto lhs_it = this->arg_values.begin();
    auto rhs_it = rhs.arg_values.begin();

    while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
    {
        if (*lhs_it != *rhs_it)
            return false;
        ++lhs_it;
        ++rhs_it;
    }

    if (this->truth != rhs.get_truth()) // fixed
        return false;

    return true;
}

string GroundedCondition::toString() const
{
    string temp = "";
    temp += this->predicate;
    temp += "(";
    for (string l : this->arg_values)
    {
        temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
}




Condition::Condition(string pred, list<string> args, bool truth)
{
    this->predicate = pred;
    this->truth = truth;
    for (string ar : args)
    {
        this->args.push_back(ar);
    }
}

string Condition::get_predicate() const
{
    return this->predicate;
}

list<string> Condition::get_args() const
{
    return this->args;
}

bool Condition::get_truth() const
{
    return this->truth;
}

bool Condition::operator==(const Condition& rhs) const // fixed
{

    if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
        return false;

    auto lhs_it = this->args.begin();
    auto rhs_it = rhs.args.begin();

    while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
    {
        if (*lhs_it != *rhs_it)
            return false;
        ++lhs_it;
        ++rhs_it;
    }

    if (this->truth != rhs.get_truth())
        return false;

    return true;
}

string Condition::toString() const
{
    string temp = "";
    if (!this->truth)
        temp += "!";
    temp += this->predicate;
    temp += "(";
    for (string l : this->args)
    {
        temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
}



Action::Action(string name, list<string> args,
    unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
    unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
{
    this->name = name;
    for (string l : args)
    {
        this->args.push_back(l);
    }
    for (Condition pc : preconditions)
    {
        this->preconditions.insert(pc);
    }
    for (Condition pc : effects)
    {
        this->effects.insert(pc);
    }
}
string Action::get_name() const
{
    return this->name;
}
list<string> Action::get_args() const
{
    return this->args;
}
unordered_set<Condition, ConditionHasher, ConditionComparator> Action::get_preconditions() const
{
    return this->preconditions;
}
unordered_set<Condition, ConditionHasher, ConditionComparator> Action::get_effects() const
{
    return this->effects;
}

bool Action::operator==(const Action& rhs) const
{
    if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
        return false;

    return true;
}

string Action::toString() const
{
    string temp = "";
    temp += this->get_name();
    temp += "(";
    for (string l : this->get_args())
    {
        temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
}


GroundedAction::GroundedAction(string name, list<string> arg_values)
{
    this->name = name;
    for (string ar : arg_values)
    {
        this->arg_values.push_back(ar);
    }
}

string GroundedAction::get_name() const
{
    return this->name;
}

list<string> GroundedAction::get_arg_values() const
{
    return this->arg_values;
}

bool GroundedAction::operator==(const GroundedAction& rhs) const
{
    if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
        return false;

    auto lhs_it = this->arg_values.begin();
    auto rhs_it = rhs.arg_values.begin();

    while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
    {
        if (*lhs_it != *rhs_it)
            return false;
        ++lhs_it;
        ++rhs_it;
    }
    return true;
}


string GroundedAction::toString() const
{
    string temp = "";
    temp += this->name;
    temp += "(";
    for (string l : this->arg_values)
    {
        temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
}



void Env::remove_initial_condition(GroundedCondition gc)
{
    this->initial_conditions.erase(gc);
}
void Env::add_initial_condition(GroundedCondition gc)
{
    this->initial_conditions.insert(gc);
}

unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> 
    Env::get_initial_condition() const
{
    return initial_conditions;
}

unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> 
    Env::get_goal_condition() const
{
    return goal_conditions;
}

void Env::add_goal_condition(GroundedCondition gc)
{
    this->goal_conditions.insert(gc);
}
void Env::remove_goal_condition(GroundedCondition gc)
{
    this->goal_conditions.erase(gc);
}
void Env::add_symbol(string symbol)
{
    symbols.insert(symbol);
}
void Env::add_symbols(list<string> symbols)
{
    for (string l : symbols)
        this->symbols.insert(l);
}
void Env::add_action(Action action)
{
    this->actions.insert(action);
}

unordered_set<Action, ActionHasher, ActionComparator> Env::get_actions(){
    return actions;
}

Action Env::get_action(string name)
{
    for (Action a : this->actions)
    {
        if (a.get_name() == name)
            return a;
    }
    throw runtime_error("Action " + name + " not found!");
}

unordered_set<string> Env::get_symbols() const
{
    return this->symbols;
}



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

double TreeNode::getH() const{

    return H_val;
}

bool TreeNode::isExpanded() const{

    return expanded;
}

vector<GroundedAction> TreeNode::getNextAction() const{

    return nextActions;
}

void TreeNode::setCondition(unordered_set<GroundedCondition, 
        GroundedConditionHasher, 
        GroundedConditionComparator> conditionsIn){

    conditions = conditionsIn;
}

void TreeNode::addSuccesor(TreeNode* succIn){

    succesors.push_back(succIn);
}

void TreeNode::addNextAction(GroundedAction actionIn){

    nextActions.push_back(actionIn);
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

void TreeNode::calcH(const unordered_set<GroundedCondition, GroundedConditionHasher, 
    GroundedConditionComparator>& goalCondsIn){

    int overlap=0;
    for(auto i_goal : goalCondsIn){

        if(conditions.find(i_goal)==conditions.end())
            overlap++;       
    }

    H_val = (double) overlap;
}

void TreeNode::expand(){expanded = true;}

void TreeNode::setNextActions(vector<GroundedAction> actionsIn){

    nextActions = actionsIn;
}

void calcSuccesors(Env* envIn, priority_queue< TreeNode*, vector<TreeNode*>, CompareF >& open_setIn , 
    TreeNode* &tempPtrIn ){

    int numSym = envIn->get_symbols().size();
    auto currConditions = tempPtrIn->getCondition();

    // cout<<"\nIncoming conditions are ";
    // for(auto i : currConditions){cout<<i<<", ";}
    // cout<<endl;

    for(auto i_action : envIn->get_actions()){

        auto actionArgCombinations = combinationCalc(envIn->get_symbols(), i_action.get_args().size());

        // auto argsTemp = i_action.get_args();
        // auto precondTemp = i_action.get_preconditions();

        for(auto i_args : actionArgCombinations){

            GroundedAction grdAcTemp( i_action.get_name(), i_args );
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> precondsTemp;
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effectsTemp;
            
            bool distinct=true;

            unordered_set<GroundedCondition, GroundedConditionHasher, 
                GroundedConditionComparator> succesorTemp = currConditions;

            auto argsAction = i_action.get_args();

            for(auto i_precond : i_action.get_preconditions()){
                
                auto argsPreCond = i_precond.get_args();
                // auto commonIndices
                // cout<<"argsAction size is "<<argsAction.size()<<endl;
                // cout<<"i_args size is "<<i_args.size()<<endl;
                list<string> stringInd = calcOverlap(argsPreCond, argsAction, i_args, envIn->get_symbols());

                // if(!distinctElems(stringInd)){
                //     distinct = false;
                //     break;
                // }            
                // vector<string> stringInd;
                // for(auto i_temp : commonIndices){
                //     stringInd.push_back(i_args[i_temp]);
                // }                
                GroundedCondition precondNew( i_precond.get_predicate(), stringInd );
                // cout<<"Final condition is "<<precondNew<<endl;
                precondsTemp.insert(precondNew);
            }

            // if(!distinct)
            //     continue;

            for(auto i_effect : i_action.get_effects()){

                auto argsEffect = i_effect.get_args();
                list<string> stringInd = calcOverlap(argsEffect, argsAction, i_args, envIn->get_symbols());

                // if(!distinctElems(stringInd)){
                //     distinct = false;
                //     break;
                // }
                //  stringInd;
                // for(auto i_temp : commonIndices){
                //     stringInd.push_back(i_args[i_temp]);
                // }
                GroundedCondition effectNew( i_effect.get_predicate(), stringInd, i_effect.get_truth() );
                // cout<<"Final effect is "<<effectNew<<", truth is "<<i_effect.get_truth()<<endl;
                effectsTemp.insert(effectNew);
            }

            // if(!distinct)
            //     continue;
            // cout<<"Preconditions before checking validity are ";
            // for(auto i : precondsTemp){cout<<i<<", ";}
            // cout<<endl;

            bool validAction = precondCheck( precondsTemp, currConditions );
            
            if(validAction && distinct){

                // cout<<"Action is valid"<<endl;

                // auto itSet = effectsTemp.begin();

                for(auto i_set : effectsTemp){

                    if(i_set.get_truth()){

                        // cout<<"Condition inserted is "<<i_set<<endl;
                        succesorTemp.insert(i_set);
                        // itSet++;                        
                    }
                    else{
                    
                        // auto tempIt = succesorTemp.find(*itSet);
                        auto temp_set = i_set;
                        temp_set.set_truth(true);

                        auto itSucc = succesorTemp.begin();
                        while(itSucc!=succesorTemp.end()){

                            if((*itSucc) == temp_set){
                                // cout<<"Condition erased is "<<*itSucc<<endl;
                                succesorTemp.erase(itSucc);                                
                                break;
                            }
                            else
                                itSucc++;
                        }                       
                    }
                }

                // add to tree
                TreeNode* succNode = new TreeNode;
                succNode->setParent(tempPtrIn);
                succNode->setCondition(succesorTemp);
                succNode->setG( tempPtrIn->getG() + 1 );
                succNode->calcH(envIn->get_goal_condition());
                tempPtrIn->addSuccesor(succNode);
                tempPtrIn->addNextAction(grdAcTemp);
                // cout<<"next action added is "<<grdAcTemp<<endl;

                open_setIn.push(succNode);

                // cout<<"Outgoing conditions are ";
                // for(auto i : succesorTemp){cout<<i<<", ";}
                // cout<<endl;
            }
        }
    }
}


bool distinctElems(list<string> candidate) 
{ 
  
    // Put all array elements in a map 
    unordered_set<string> s; 

    for (auto i : candidate) { 
        
        s.insert(i); 
    } 
  
    // If all elements are distinct, size of 
    // set should be same array. 
    return (s.size() == candidate.size()); 
} 

bool precondCheck(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>
    &precondsTempIn, const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>&
    currConditionsIn )
{   
    // cout<<"Current conditions are "<<currConditionsIn<<endl;
    for(auto i_cond : precondsTempIn){

        // cout<<"precondition is "<<i_cond<<endl;
        if(currConditionsIn.find(i_cond) == currConditionsIn.end())
            return false;
    }

    return true;
}

list<string> calcOverlap(list<string> smaller, list<string> larger, list<string> argsIn, 
    unordered_set<string> symbolsIn){ 
    
    list<string> stringOut;
    auto it_arg = argsIn.begin();
    // cout<<"\nsmaller string is ";
    // for(auto i : smaller){cout<<i<<", ";}
    // cout<<endl;

    // cout<<"\n larger string is ";
    // for(auto i:larger){cout<<i<<", ";}
    // cout<<endl;

    // cout<<"\n argsIn string is ";
    // for(auto i:argsIn){cout<<i<<", ";}
    // cout<<endl;

    for(auto i: smaller){

        // cout<<"\n smaller elem is "<<i<<endl;
        bool found = false;
        
        for(auto temp : symbolsIn){

            if(temp == i){
                found = true;
                stringOut.push_back(i);
                break;
            }
        }

        if(!found){
         
            int jCt=0;

            for(auto j : larger){

                if(j==i){                
                    break;
                }
                jCt++;
            }
            // cout<<"iterator is "<<jCt<<endl;
            
            if(jCt<larger.size()){
                // int itTemp = it - larger.cbegin()
                // cout<<"itTemp is "<<jCt<<endl;
                auto it_arg = next(argsIn.begin(), jCt);
                stringOut.push_back( *it_arg );
            }
            else
                cout<<"no argument found"<<endl;        
        }
    }
    
    // cout<< "end result string is ";
    // for(auto i:stringOut){cout<<i<<", ";}
    // cout<<endl;

    return stringOut;
} 

vector< list<string> > combinationCalc( const unordered_set<string>& symbolsIn, int K ){

    vector< list<string> > actionArgCombinations;
    int N = symbolsIn.size();
    string bitmask(K, 1); // K leading 1's
    bitmask.resize(N, 0); // N-K trailing 0's

    // print integers and permute bitmask
    int combnCt =0;
    do {

        int symCt=0;
        list<string> listTemp;
        for (auto i_string : symbolsIn){

            if (bitmask[symCt]){

                listTemp.push_back(i_string);
                // cout<< "symbol inserted is "<<i_sym<<endl;
            }

            symCt++;
        }

        // cout<<"Before Permutation list of strings is ";
        // for(auto i : listTemp){cout<<i<<", ";}
        // cout<<endl;

        findPermutations(listTemp, actionArgCombinations);

        // actionArgCombinations.push_back(listTemp);
        combnCt++;

    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

    return actionArgCombinations;
}

void findPermutations(list<string> smaller, vector<list<string> >& combinationVect ) 
{ 
    // Sort the given array 
    smaller.sort();

    do { 
        combinationVect.push_back(smaller);

    } while (next_permutation(smaller.begin(), smaller.end())); 
}