#include <stdio.h>
#include <math.h>
#include "header.h"

// Joseph Garcia
// ECE 417
// Laboratory 3  (Matrix Operation Subroutines)
// 10/22/2014   `
/*-----------------------------------------------------------------------------------------------------------------*/
//HELPFUL DIAGRAMS REPRESENTING THE MATRIX M TRADITIONALLY WRITTEN, AND THE MATRIX M AS IT IS STORED IN A 2D ARRAY

//Matrix (traditionally written    homog. point as a column vector        Result
//
//[M_11 M_12 M_13  M_14]    [P_x]                                     [M11*P_x +M12*P_y + M13*P_z + M14*1]
//[M_21 M_22 M_23  M_24]  * [P_y]               =                    [M21*P_x +M22*P_y + M23*P_z + M24*1]
//[M_31 M_32 M_33  M_34]    [P_z]                                     [M31*P_x +M32*P_y + M33*P_z + M34*1]
//[M_41 M_42 M_43  M_44]    [   1 ]                                     [M41*P_x +M42*P_y + M43*P_z + M44*1]
//
//Matrix as stored in a 2D array              homog. point as an array
//
//i->          0    1    2    3                0    1    2      3
//     0     [M_11 M_21 M_31 M_41]           [P_x, P_y, P_z, 1 (one)]
//     1     [M_12 M_22 M_32 M_42]
//     2     [M_13 M_23 M_33 M_43]
//j^   3     [M_14 M_24 M_34 M_44]

//For this lab you will program the forward kinematics for the Lab-Volt robot. In addition to the routines
//developed in Lab 2, you are required to write a new routine which will
//    - accept the joint parameters d, q, a, and alpha (as an input)    DONE
//   - produce the corresponding 4x4 homogeneous matrix as output.     DONE
//
//Your main program should :
//    accept the five joint angles as input    DONE
//        input the five joint angle (theta1, theta2, theta3, theta4, theta5)  DONE
//        theta1 goes into 0t1, theta2 goes into 1t2, theta3 goes into 2t3, theta4 goes into 3t4, theta5 goes into 4t5
//        for each link it should call your new routine using stored values of di, ai, and alpha_i along with the input joint angle qi.
//        - put stuff in the proper matrices, as noted above.
//
//The five 4x4 homogeneous matrices produced should then be multiplied together
//
//    - the result,0T5, should be printed on the screen.
//
//(N.B.) The joint parameters and definitions of joint angles should be the same as those given in the solution to the
//     homework.

//function prototypes up here
void create_general_matrix(double general_matrix[4][4], double d_i, double a_i, double alpha_i, double theta_i);
void matrix_multiply(double A[4][4],double B[4][4],double C[4][4]);
void print_matrix(double M[4][4]);
void display_matrices(double M1[4][4], double M2[4][4], double M3[4][4], double M4[4][4], double M5[4][4], double M6[4][4]);
void convert_input_to_rad(double * theta_1,double * theta_2,double * theta_3,double * theta_4,double * theta_5);

void main(void)
{
    printf("Welcome to the Forward Kinematics Matrix Multiplier Application.\n Press A to enter values for theta. Press Q to quit.\n\n");

    double theta_1 = 0, theta_2 = 0, theta_3 = 0, theta_4 = 0, theta_5 = 0;
    double theta_4_prime = 0.0;
    char letterchoice = 0;

    double one_wrt_zero[4][4];   // 0^T_1
    double two_wrt_one[4][4];    //1^T_2
    double three_wrt_two[4][4];   //2^T_3
    double four_wrt_three[4][4];  //3^T_4
    double five_wrt_four[4][4];   //4^T_5
    double five_wrt_zero[4][4];    // O^T_5
    double C[4][4];               // general matrix product answer storing variable.
    double C1[4][4];
    double C2[4][4];

    do
    {
        printf("Pick your letter: ");
        scanf(" %c", &letterchoice);    // NOTE: DO NOT GET RID OF THE SPACE!!!!
        switch(letterchoice)
        {
            case 7:
            {
                    printf("\n");
                    printf("Please input the five angles that you would like to compute a 0T5 matrix for: ");
                    scanf("%lf %lf %lf %lf %lf", &theta_1, &theta_2, &theta_3, &theta_4, &theta_5);
                    printf("\n");
                    //transform the input variables to radians....
                    convert_input_to_rad(&theta_1, &theta_2, &theta_3, &theta_4, &theta_5);
                    //Create the actual transformation matrices
                    create_general_matrix(one_wrt_zero,d1,a1,alpha_1, theta_1);    //create 0T1
                    create_general_matrix(two_wrt_one,d2,a2,alpha_2, theta_2);     //Create 1T2
                    create_general_matrix(three_wrt_two, d3, a3, alpha_3, theta_3);  //Create 2T3
                    theta_4_prime = theta_4 + 1.5708;
                    create_general_matrix(four_wrt_three,d4, a4, alpha_4, theta_4_prime);   //Create 3T4
                    create_general_matrix(five_wrt_four, d5, a5, alpha_5, theta_5);      //Create 4T5

                    //    multiply the five matrices together
                    matrix_multiply(one_wrt_zero,two_wrt_one,C); //    0T1 * 1T2 = 0T2
                    matrix_multiply(C,three_wrt_two,C1);    //  (0T1 * 1T2) * 2T3 = 0T3
                    //something is happening from directly below this line and beyond.
                    matrix_multiply(C1,four_wrt_three,C2);    //  ((0T1 * 1T2) * 2T3) * 3T4 = 0T4
                    matrix_multiply(C2,five_wrt_four,five_wrt_zero);   //  (((0T1 * 1T2) *2T3) * 3T4) * 4T5 = 0T5
                    printf("Here is the 0T5 Matrix\n \n");print_matrix(five_wrt_zero);
                    printf("\n");
            }
            break;

            case 'B':
                display_matrices(one_wrt_zero,two_wrt_one,three_wrt_two,four_wrt_three,five_wrt_four,five_wrt_zero);
            break;
        }
    }while(letterchoice != 'Q');
}

//function implementations down here

//void create_general_matrix(double general_matrix[4][4], double d_i, double a_i, double alpha_i, double theta_i)
//{
//    general_matrix[0][0] = cos(theta_i);
//    general_matrix[0][1] = -cos(alpha_i)*sin(theta_i);
//    general_matrix[0][2] = sin(alpha_i)*sin(theta_i);
//    general_matrix[0][3] = a_i*cos(theta_i);
//    general_matrix[1][0] = sin(theta_i);
//    general_matrix[1][1] = cos(alpha_i)*cos(theta_i);
//    general_matrix[1][2] = -sin(alpha_i)*cos(theta_i);
//    general_matrix[1][3] = a_i*sin(theta_i);
//    general_matrix[2][0] = 0;
//    general_matrix[2][1] = sin(alpha_i);
//    general_matrix[2][2] = cos(alpha_i);
//    general_matrix[2][3] = d_i;
//    general_matrix[3][0] = 0;
//    general_matrix[3][1] = 0;
//    general_matrix[3][2] = 0;
//    general_matrix[3][3] = 1;
//}
//
//void matrix_multiply(double A[4][4],double B[4][4],double C[4][4])
//{
//  // printf("\n");
//    const int size = 4;         // stylistic choice to use a constant equal to 4 as the order of the homogeneous matrix
//    int i = 0,j = 0,k = 0;         // initialize matrix looping variables
//    double sum = 0.0;               // initialize the sum which represents C_ij (C[i-1][j-1])
//
//    for( i = 0; i < size; i++)           // pick the ith row of matrix A (A[i][0,1,2,3])
//    {
//        for(k = 0; k < size; k++)
//        {
//            sum = 0;                                 // reinitialize the sum representing C_ik for every new combination of i and k
//            for(j = 0; j < size; j++)
//            {
//                sum = sum + A[i][j] * B[j][k];        // sum the product of the ith row of A times the kth column of B to compute C[i][k]
//            }
//            C[i][k] = sum;
//        }
//    }
//    //print_matrix(C);     // print out the product C = A*B
// //   printf("\n");
//}
//
//void print_matrix(double M[4][4])  // print out a general 4x4 matrix, in traditional Matrix format
//{
//    int i =0, j =0;    // initialize matrix counting variables
//    for(i = 0; i < 4; i++)
//    {
//        for(j =0; j < 4; j++)
//        {
//            printf("%.3f ", M[i][j]);   // print out M[i][j]
//        }
//        printf("\n");
//    }
//}
//
//void display_matrices(double M1[4][4], double M2[4][4], double M3[4][4], double M4[4][4], double M5[4][4], double M6[4][4])
//{
//    printf("Here are the five transformation matrices.\n");
//
//    printf("0T1\n");
//    print_matrix(M1);
//    printf("\n");
//
//    printf("1T2\n");
//    print_matrix(M2);
//     printf("\n");
//
//     printf("2T3\n");
//    print_matrix(M3);
//     printf("\n");
//
//     printf("3T4\n");
//    print_matrix(M4);
//     printf("\n");
//
//     printf("4T5\n");
//    print_matrix(M5);
//     printf("\n");
//
//     printf("5T0\n");
//    print_matrix(M6);
//    printf("\n");
//}
//
//void convert_input_to_rad(double * theta_1,double * theta_2,double * theta_3,double * theta_4,double * theta_5)
//{
//    const double PI = 3.14159;
//    *theta_1 = *theta_1*PI/180;
//    *theta_2 = *theta_2*PI/180;
//    *theta_3 = *theta_3*PI/180;
//    *theta_4 = *theta_4*PI/180;
//    *theta_5 = *theta_5*PI/180;
//}
