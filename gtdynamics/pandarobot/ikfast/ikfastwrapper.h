/*
 * THIS IS A MODIFIED VERSION OF THE FOLLOWING:
 * 
 * IKFast Demo
 * 
 * Shows how to calculate FK from joint angles.
 * Calculates IK from rotation-translation matrix, or translation-quaternion pose.
 * Performance timing tests.
 *
 * Run the program to view command line parameters.
 * 
 * 
 * To compile, run:
 * g++ -lstdc++ -llapack -o compute ikfastdemo.cpp -lrt
 * (need to link with 'rt' for gettime(), it must come after the source file name)
 *
 * 
 * Tested with Ubuntu 11.10 (Oneiric)
 * IKFast54 from OpenRAVE 0.6.0
 * IKFast56/61 from OpenRave 0.8.2
 *
 * Author: David Butterworth, KAIST
 *         Based on code by Rosen Diankov
 * Date: November 2012
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IKFAST_NAMESPACE pandainternal


#define IK_VERSION 61
#include "ikfast61_panda.cpp"

//----------------------------------------------------------------------------//

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

using IKFAST_NAMESPACE::IkReal;


namespace panda { 
    using ikvector = std::vector<IkReal>;

    class IkFastWrapper { 
        public:
            int num_joints, num_free_params; //should be static
            IkFastWrapper(); 
            ikvector forward(ikvector joint_config);
            std::vector<ikvector> inverse(ikvector ee_pose, ikvector vfree);
    }; 

    IkFastWrapper::IkFastWrapper() { 
            num_joints = IKFAST_NAMESPACE::GetNumJoints();
            num_free_params = IKFAST_NAMESPACE::GetNumFreeParameters();
    } 

    /**
     * @brief Forward IkFastWrapper on Panda robot using IKFast.
     * 
     * @param joint_config 
     * @return std::vector<IkReal> 
     */
    ikvector IkFastWrapper::forward(ikvector joint_config) {

        // Probably should be an assert
        if( joint_config.size() != num_joints ) {
            std::cout << "\nError: (forward IkFastWrapper) expects vector of "<< num_joints << " values describing joint angles (in radians).\n\n";
            return ikvector(0);
        }

        // This will be changed once completely integrated
        // Put input joint values into array, could be better
        IkReal joints[num_joints];
        for (int i=0; i<num_joints; i++)
        {
            joints[i] = joint_config[i];
        }

        IkReal eerot[9],eetrans[3];
        IKFAST_NAMESPACE::ComputeFk(joints, eetrans, eerot);


        // This will be changed once completely integrated
        // Join rotation matrix and position vector into pose vector
        ikvector ee_pose(12);
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++)
                ee_pose[4*i+j] = eerot[i*3+j];
            ee_pose[4*i+3] = eetrans[i];
        }

        return ee_pose;
    }
    
    /**
     * @brief Inverse IkFastWrapper on Panda robot using IKFast.
     * 
     * @param ee_pose 
     * @param vfree 
     * @return std::vector<std::vector<IkReal>>
     */
    std::vector<ikvector> IkFastWrapper::inverse(ikvector ee_pose, ikvector vfree) {
        

        // This will be changed once completely integrated
        // Probably should be an assert
        if (!(ee_pose.size() == 12 && vfree.size()==num_free_params)){
                    std::cout << "\nError: (inverse IkFastWrapper) please specify transformation of end effector with one of the following formats:\n"
                  << "    1) A vector of 7 values: a 3x1 translation (tX), and a 1x4 quaternion (w + i + j + k)\n"
                  <<  "    2) A (row-major) vector of 12 values: a 3x4 rigid transformation matrix with a 3x3 rotation R (rXX), and a 3x1 translation (tX)\n\n" ;
            return std::vector<ikvector>();
        }

        // This will be changed once completely integrated
        // Separate pose into arrays: rotation matrix and position vector
        IkReal eerot[9],eetrans[3];
        for (unsigned int i=0; i<3; i++) {
            for (int j=0; j<3; j++)
                eerot[i*3+j] = ee_pose[4*i+j];
            eetrans[i] = ee_pose[4*i+3];
        }

        // Ikfast class where solutions are stored
        IkSolutionList<IkReal> solutions;

        bool bSuccess = IKFAST_NAMESPACE::ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

        if( !bSuccess ) {
            fprintf(stderr,"Error: (inverse IkFastWrapper) failed to get ik solution\n");
            return std::vector<ikvector>();
        }

        int num_sols = (int)solutions.GetNumSolutions();

        // This will be changed once completely integrated
        std::vector<ikvector> joint_configs(num_sols, ikvector(num_joints));
        for(int i = 0; i < num_sols; ++i) {
            const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);

            int this_sol_free_params = (int)sol.GetFree().size();
            //cout<<"sol"<<i<<" (free="<<this_sol_free_params<<"): ";

            ikvector solvalues(num_joints);
            ikvector vsolfree(this_sol_free_params, 0);
            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

            for(int j = 0; j < solvalues.size(); ++j)
                joint_configs[i][j] =solvalues[j];
        }

        return joint_configs;
    }
}
