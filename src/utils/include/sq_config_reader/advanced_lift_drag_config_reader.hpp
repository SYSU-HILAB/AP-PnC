#ifndef SQ_CONFIG_READER_ADVANCED_LIFT_DRAG_CONFIG_READER_HPP_
#define SQ_CONFIG_READER_ADVANCED_LIFT_DRAG_CONFIG_READER_HPP_

#include <sq_config_reader/sq_config_reader.hpp>

namespace sq_config_reader
{
    class AdvancedLiftDragConfigReader : public SQLiteConfigReader
    {
       public:
        AdvancedLiftDragConfigReader();

        // Getters for the configuration values
        double get_sigmoid_blend() const { return sigmoid_blend_; }
        double get_cl_alpha_0() const { return cl_alpha_0_; }
        double get_cl_alpha() const { return cl_alpha_; }
        double get_alpha_stall() const { return alpha_stall_; }
        double get_eff() const { return eff_; }
        double get_cd_0() const { return cd_0_; }
        double get_cd_flat_plate() const { return cd_flat_plate_; }
        double get_cy_beta() const { return cy_beta_; }
        double get_cl_beta_loss() const { return cl_beta_loss_; }
        double get_scale_factor() const { return scale_factor_; }

       protected:
        bool prepare_statement(int id) override;
        bool fetch_data() override;

       private:
        // Configuration values
        double sigmoid_blend_;  // Sigmoid blending parameter
        double cl_alpha_0_;     // Initial lift coefficient
        double cl_alpha_;       // Lift curve slope (per radian)
        double alpha_stall_;    // Stall angle (radians)
        double eff_;            // Wing efficiency factor
        double cd_0_;           // Zero-lift drag coefficient
        double cd_flat_plate_;  // Flat plate drag coefficient
        double cy_beta_;        // Side force derivative
        double cl_beta_loss_;   // Lift loss factor with sideslip
        double scale_factor_;   // Scale factor for aerodynamic forces
    };

}  // namespace sq_config_reader

#endif  // SQ_CONFIG_READER_ADVANCED_LIFT_DRAG_CONFIG_READER_HPP_