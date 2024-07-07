
namespace Controllers
{
    class PID 
    {
        public:
            PID(double kp, double kd, double ki);
            double compute_adjustment(double current_heading);
            void set_heading_goal(double current_heading);
            double get_error();

        private:
            double KP;
            double KD;
            double KI;
            double last_error;
            double heading_goal; 
    };
}