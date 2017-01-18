#include "stdio.h"
#include <iostream>
#include "header.h"
#include "robotics_funcs_consts.h"

// Joseph Garcia
// ECE 417
// Master Code for Lab-Volt Robot Mathematics
// 10/8/2014

//The purpose of this lab is to develop a few subroutines (in C or C++) which will be used in future lab assignments.  The subroutines should be written
//so they are easily used in future programs.  They should be well commented.  The following subroutines should be written:
//
//    A subroutine which will multiply two homogeneous matrices to form a composite homogeneous matrix; i.e., matrix A and matrix B are input
//            and matrix C is computed as A times B.    (function: matrix_multiply)
//    A subroutine which will transform a point using a given homogeneous matrix; i.e., matrix R and point pUVW
//            (not augmented with a 1) are input and pXYZ  is computed as R times pUVW. (transform_point function)
//    A subroutine to compute the inverse of a homogeneous transformation matrix;  i.e., matrix R is input and matrix R-1 is computed. (function: invert_matrix)
//
//Note with each of these subroutines you may assume that homogeneous 4x4 matrices are involved and can therefore use the appropriate simplifications.
//
//You are also required to write a main program which inputs matrix values from the keyboard and calls each of the subroutines to test them.
//The following is suggested, but variations are allowed (e.g., menu driven operation):
//1) Input matrix A. (function: input_matrix)
//2) Input matrix B. (function: input_matrix)
//3) Input point pUVW. (function:input_point)
//4) Print A times B. (function: matrix_multiply)
//5) Print A-1.    (function: invert_matrix)
//6) Print A times  pUVW.  Note that printing A times A-1 is also a good test. (function: transform_point)

//Note: Every time a new matrix (A,B,R) or point (point)  is needed, the user must go to the appropriate function (i.e. input matrix and input_point, respectively)

/*-----------------------------------------------------------------------------------------------------------------*/
//HELPFUL DIAGRAMS REPRESENTING THE MATRIX M TRADITIONALLY WRITTEN, AND THE MATRIX M AS IT IS STORED IN A 2D ARRAY

//Matrix (traditionally written    homog. point as a column vector        Result
//
//[M_11 M_12 M_13  M_14]    [P_x]                                     [M11*P_x +M12*P_y + M13*P_z + M14*1]
//[M_21 M_22 M_23  M_24]  * [P_y]               =                    [M21*P_x +M22*P_y + M23*P_z + M24*1]
//[M_31 M_32 M_33  M_34]    [P_z]                                     [M31*P_x +M32*P_y + M33*P_z + M34*1]
//[M_41 M_42 M_43  M_44]    [   1 ]                                     [M41*P_x +M42*P_y + M43*P_z + M44*1]
//
//Matrix as stored in a 2D array              h;mog. point as an array
//
//i->          0    1    2    3                0    1    2      3
//     0     [M_11 M_21 M_31 M_41]           [P_x, P_y, P_z, 1 (one)]
//     1     [M_12 M_22 M_32 M_42]
//     2     [M_13 M_23 M_33 M_43]
//j^   3     [M_14 M_24 M_34 M_44]

/*-----------------------------------------------------------------------------------------------------------------*/

//void matrix_multiply(double A[4][4],double B[4][4], double C[4][4]);    /*Compute and print out A*B = C*/
//void transform_point(double M[4][4], double point[4], double point_tfmd[4]);  /*User inputs UVW, function autofills 1 at the end for augmented vector*/
//void invert_matrix(double M[4][4], double M_1[4][4]);   /*Invert any 4x4 matrix*/
//void input_matrix(double M[4][4]);      /*input a homogeneous matrix*/
//void input_point(double point[4]);      /*input a set of 3D points,this function must be called every time you desire a new point to transform.*/
//void print_matrix(double M[4][4]);       /*general function to print out a matrix*/

int main(void)
{
    robotics_funcs_consts robo_math;       // create an object of the robotics_funcs_consts class
    int numberchoice=0;                    // initialize variable used for holding user input from menu selection
    char matrix_choice1, matrix_choice2;    // used to determine the matrices into which values will be stored (e.g. matrix_choice1 = 'A', matrix_choice2 = 'B'
    double point[4] = {0,0,0,0};    // initialize the homogeneous coordinates which the user can modify at any time.
    double point_tfmd[4] = {0,0,0,0};   // the transformed homogeneous coordinate (transformed by the function transform_point)
    double A[4][4];            //Matrix A
    double B[4][4];            //Matrix B
    double C[4][4];             //Used as a general answer storage matrix for matrix multiplication (matrix_multiply)
    double R[4][4];             // a general transformation matrix

    double theta_1 = 0, theta_2 = 0, theta_3 = 0, theta_4 = 0, theta_5 = 0;
    double theta_4_prime = 0.0;

    double one_wrt_zero[4][4];   // 0^T_1
    double two_wrt_one[4][4];    //1^T_2
    double three_wrt_two[4][4];   //2^T_3
    double four_wrt_three[4][4];  //3^T_4
    double five_wrt_four[4][4];   //4^T_5
    double five_wrt_zero[4][4];    // O^T_5
    double C1[4][4];
    double C2[4][4];

    printf("welcome to the Robotics Matrix Operations Calculator. When entering matrix values, make sure to list them as such: M_11 M_12, M_13, M_14, M_21, M_22, etc.\n\n");
    do
    {
        std::cout << "Enter the Code for the Matrix Operation you would like to perform:\n 1: Input Homogeneous Matrix\n 2: Input a point in 3D space\n 3: Multiply Homogeneous Matrices\n";
        std::cout << " 4: Invert a Homogeneous Matrix\n 5: Transform a Point\n 6: View a Matrix\n 7: Do Forward Kinematics\n 8: Display N_T_N-1 matrices\n";
        std::cout << " 9: Do Inverse Kinematics\n 0: Quit the program\n";
        std::cout << "For Procedure (8), please enter the five theta angles in degrees.\n";
        std::cin >> numberchoice;
        switch(numberchoice)
        {
            case 1:               // input the first twelve values of a homogeneous matrix (A,B,C, or R). the program will autofill the bottom row of the matrix for the user
                std::cout << "Input Matrix A, B, or R. Only twelve values are accepted into the homogeneous matrix." << std::endl << std::endl;
                std::cin >> matrix_choice1;            // additional scanf statement to get around the carriage return being fed into the scanf function
                printf("\n");
                if(matrix_choice1 == 'A')  robo_math.input_matrix(A);
                if(matrix_choice1 == 'B')  robo_math.input_matrix(B);                 // use the input_matrix function to fill the desired matrix
                if(matrix_choice1 == 'R')  robo_math.input_matrix(R);
                printf("\n");
            break;

            case 2:                // input a point which will be made homogeneous
                robo_math.input_point(point);
            break;

            case 3:          // multiply two homogeneous matrices of the user's choosing, which will be stored into general result matrix C
                std::cout << "Choose the two matrices, their product will be stored in a Matrix C." << std::endl << std::endl;
                std::cin >> matrix_choice2;
                if(matrix_choice1 == 'A' && matrix_choice2 == 'B') robo_math.matrix_multiply(A,B,C);
                if(matrix_choice1 == 'A' && matrix_choice2 == 'R') robo_math.matrix_multiply(A,R,C);
                if(matrix_choice1 == 'B' && matrix_choice2 == 'A') robo_math.matrix_multiply(B,A,C);   // doing all 6 permutations of matrix multiplication as matrix multiplication
                if(matrix_choice1 == 'B' && matrix_choice2 == 'R') robo_math.matrix_multiply(B,R,C);   // is generally not commutative
                if(matrix_choice1 == 'R' && matrix_choice2 == 'A') robo_math.matrix_multiply(R,A,C);
                if(matrix_choice1 == 'R' && matrix_choice2 == 'B') robo_math.matrix_multiply(R,B,C);
                printf("\n");
            break;

            case 4:            // invert a homogeneous matrix of the user's choosing
                std::cout << "Choose a matrix to invert: A,B, or R.\n" << std::endl;
                std::cin >> matrix_choice1;
                if(matrix_choice1 == 'A') robo_math.invert_matrix(A);
                if(matrix_choice1 == 'B') robo_math.invert_matrix(B);            // invert the proper matrix and store its result into the corresponding inverse matrix
                if(matrix_choice1 == 'R') robo_math.invert_matrix(R);
                printf("\n");
            break;

            case 5:                 // transform a point of the user's choosing (Note: user must go to option 1 again and input a new point if they want a new one to transform)
                std::cout << "Which matrix (A,B, or R) would you like to use to transform the point (Use Option 2 to change the point)?" << std::endl << std::endl;
                std::cin >> matrix_choice1;
                if(matrix_choice1 == 'A') robo_math.transform_point(A, point, point_tfmd);
                if(matrix_choice1 == 'B') robo_math.transform_point(B, point, point_tfmd); // transform the point using the desired matrix and store the transformed matrix into point_tfmd
                if(matrix_choice1 == 'R') robo_math.transform_point(R, point, point_tfmd);
                break;
            case 6:
                printf("Which matrix would you like to view? A, B, or R?");
                std::cin >> matrix_choice1;
                if(matrix_choice1 == 'A') robo_math.print_matrix(A);
                if(matrix_choice1 == 'B') robo_math.print_matrix(B); // transform the point using the desired matrix and store the transformed matrix into point_tfmd
                if(matrix_choice1 == 'R') robo_math.print_matrix(R);
            break;
            case 7:
            {
                printf("\n");
                printf("Please input the five angles that you would like to compute a 0T5 matrix for: ");
                std::cin >> theta_1 >> theta_2 >> theta_3  >> theta_4 >> theta_5;
                std::cout << std::endl;
                //troboransform the input variables to radians....
                robo_math.convert_input_to_rad(&theta_1, &theta_2, &theta_3, &theta_4, &theta_5);
                //Create the actual transformation matrices
                robo_math.create_general_matrix(one_wrt_zero,d1,a1,alpha_1, theta_1);    //create 0T1
                robo_math.create_general_matrix(two_wrt_one,d2,a2,alpha_2, theta_2);     //Create 1T2
                robo_math.create_general_matrix(three_wrt_two, d3, a3, alpha_3, theta_3);  //Create 2T3
                theta_4_prime = theta_4 + alpha_1;          //alpha_1 = 1.5707
                robo_math.create_general_matrix(four_wrt_three,d4, a4, alpha_4, theta_4_prime);   //Create 3T4
                robo_math.create_general_matrix(five_wrt_four, d5, a5, alpha_5, theta_5);      //Create 4T5

                //    multiply the five matrices together
                robo_math.matrix_multiply(one_wrt_zero,two_wrt_one,C); //    0T1 * 1T2 = 0T2
                robo_math.matrix_multiply(C,three_wrt_two,C1);    //  (0T1 * 1T2) * 2T3 = 0T3
                robo_math.matrix_multiply(C1,four_wrt_three,C2);    //  ((0T1 * 1T2) * 2T3) * 3T4 = 0T4
                robo_math.matrix_multiply(C2,five_wrt_four,five_wrt_zero);   //  (((0T1 * 1T2) *2T3) * 3T4) * 4T5 = 0T5
                std::cout << "Here is the 0T5 Matrix" << std::endl << std::endl;
                robo_math.print_matrix(five_wrt_zero);
                std::cout << std::endl;
            }
            break;
            case 8:
                robo_math.display_matrices(one_wrt_zero,two_wrt_one,three_wrt_two,four_wrt_three,five_wrt_four,five_wrt_zero);
            break;
            case 9:
                robo_math.find_all_thetas(four_wrt_three, five_wrt_four, five_wrt_zero);
            break;
        }
    } while(numberchoice != 0);   // 0 is the variable chosen which allows the user to quit the Matrix Operations Program
    return 0;
}



