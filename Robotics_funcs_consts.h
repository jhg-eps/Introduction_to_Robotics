#ifndef ROBOTICS_FUNCS_CONSTS_H
#define ROBOTICS_FUNCS_CONSTS_H


class robotics_funcs_consts
{
    public:
        robotics_funcs_consts(void);
        void matrix_multiply(double A[4][4],double B[4][4],double C[4][4]);    /*Compute and print out A*B = C*/
        void transform_point(double M[4][4], double point[4], double point_tfmd[4]);  /*User inputs UVW, function autofills 1 at the end for augmented vector*/
        void invert_matrix(double M[4][4]);   /*Invert any 4x4 matrix*/
        void input_matrix(double M[4][4]);      /*input a homogeneous matrix*/
        void input_point(double point[5]);      /*input a set of 3D points,this function must be called every time you desire a new point to transform.*/
        void print_matrix(double M[4][4]);       /*general function to print out a matrix*/
        void create_general_matrix(double M[4][4], double d_i, double a_i, double alpha_i, double theta_i);
        void display_matrices(double M1[4][4], double M2[4][4], double M3[4][4], double M4[4][4], double M5[4][4], double M6[4][4]);
        void convert_input_to_rad(double * theta_1,double * theta_2,double * theta_3,double * theta_4,double * theta_5);
        void find_all_thetas(double four_wrt_three[4][4], double five_wrt_four[4][4], double five_wrt_zero[4][4]);
        void create_zero_wrt_three_mtx(double M[4][4],double t1, double t2, double t3);
         ~robotics_funcs_consts(void);

    // function implementations to go down here

    private:
        double A[4][4];
        double B[4][4];
        double C[4][4];
        double M[4][4];
        double M_1[4][4];
        double point[5];
        double point_tfmd[4];
        double M1[4][4];
        double M2[4][4];
        double M3[4][4];
        double M4[4][4];
        double M5[4][4];
        double * theta_1;
        double * theta_2;
        double * theta_3;
        double * theta_4;
        double * theta_5;

};

#endif // 417_FUNCS_CONSTS_H
