#include <diet_estimation_skill_server/heuristics/fat_selector.h>

namespace diet_estimation_skill {

    /// <summary>
    /// Contructor
    /// </summary>
    FatSelector::FatSelector() {}

    /// <summary>
    /// Destructor
    /// </summary>
    FatSelector::~FatSelector() {}

    /// <summary>
    /// Setup static configuration for current heuristic
    /// </summary>
    /// <param name="_node_handle"> public node handle.</param>
    /// <param name="_private_node_handle"> private node handle.</param>
    /// <param name="_configuration_namespace"> ros param configuration namespace.</param>
    void FatSelector::setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,ros::NodeHandlePtr &_private_node_handle, std::string _configuration_namespace){

        _private_node_handle->param<double>(_configuration_namespace + "threshold", threshold_, 1.0);
        this->setupBaseConfigurationFromParameterServer(_node_handle, _private_node_handle, _configuration_namespace);
    }

    /// <summary>
    /// Run current heuristic
    /// </summary>
    /// <returns>true if succeeded, false if
    /// * All food candidates extrapolate the threshold
    /// </returns>
    bool FatSelector::run(){

        fat_value_arr_.clear();
        previous_scores_arr_.clear();
        resulted_scores_arr_.clear();

        ROS_INFO_STREAM("Running fat selector");

        int i = 0;

        std::vector<CandidateTuple>::iterator it;
        for (it = candidate_list_.begin(); it != candidate_list_.end(); ++it)
        {
            extrapolation_arr_.push_back(std::get<1>(*it));
            previous_scores_arr_.push_back(std::get<2>(*it));

            if( !std::get<1>(*it) ) {
                fat_value_arr_.push_back((double) std::get<0>(*it).fat_rate * std::get<0>(*it).gram);
                if(fat_value_arr_.back() < threshold_) {
                    extrapolation_arr_.back() = true;
                    std::get<1>(*it) = true;
                    fat_value_arr_.back() = 0;
                }
            }
            else
                fat_value_arr_.push_back(0);

            i++;
        }

        if(std::accumulate(extrapolation_arr_.begin(), extrapolation_arr_.end(), 0) == (int)fat_value_arr_.size()){

            ROS_ERROR_STREAM("All candidates extrapolate the Fat threshold.");
            return false;
        }

        if(!std_vector_operations::normalizeVector(fat_value_arr_))
        {
            ROS_DEBUG_STREAM("Max == Min. All candidates are eligible (before extrapolation analysis) according to the protein selector array.");
            std::vector<double>::iterator dit;
            for (dit = fat_value_arr_.begin(); dit != fat_value_arr_.end(); ++dit){
                *dit = 0;
            }
        }

        std_vector_operations::scalarMultiplicationToVector(fat_value_arr_, (weight_)); //penalty weight
        std_vector_operations::sumEachElementOfVector(fat_value_arr_, previous_scores_arr_, resulted_scores_arr_);


        if(!std_vector_operations::normalizeVector(resulted_scores_arr_))
        {
            ROS_DEBUG_STREAM("Max == Min. All candidates are eligible (before extrapolation analysis) according to the protein selector array.");
            std::vector<double>::iterator dit;
            for (dit = resulted_scores_arr_.begin(); dit != resulted_scores_arr_.end(); ++dit){
                *dit = 0;
            }
        }


        int j=0;
        for (it = candidate_list_.begin(); it != candidate_list_.end(); ++it)
        {

            if( !std::get<1>(*it) )
                std::get<2>(*it) = resulted_scores_arr_.at(j);

            j++;
           ROS_DEBUG_STREAM("Candidate "<<std::get<0>(*it).food_name << " extrapolates? : " << std::get<1>(*it) << ". Score: " << std::get<2>(*it));
        }

        return true;
    }
}