#include <diet_estimation_skill_server/diet_estimation_base.h>


#include <math.h>
#include <angles/angles.h>
#include <numeric>

#ifndef CALORIC_SELECTOR_BASE_H
#define CALORIC_SELECTOR_BASE_H


namespace diet_estimation_skill {
    class CaloricSelector : public DietEstimationBase {
    public:
        //Constructor
        CaloricSelector();
        ~CaloricSelector();

        void setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,ros::NodeHandlePtr &_private_node_handle, std::string _configuration_namespace);


    protected:

        bool run();
        double threshold_, protein_cal_per_gram_, fat_cal_per_gram_, fiber_cal_per_gram_;
        std::vector<double> caloric_value_arr_, previous_scores_arr_, resulted_scores_arr_;
        std::vector<bool> extrapolation_arr_;
    };
    
}

#endif