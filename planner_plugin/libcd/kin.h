/** \file kin.h
 * \brief Interface to cd_kin, a collection of useful routines for
 *        kinematics.
 * \author Christopher Dellin
 * \date 2010-2012
 */

/* (C) Copyright 2010-2013 Carnegie Mellon University */

/* This module (cd_kin) is part of libcd.
 *
 * This module of libcd is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This module of libcd is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * A copy of the GNU General Public License is provided with libcd
 * (license-gpl.txt) and is also available at <http://www.gnu.org/licenses/>.
 */

#ifndef CD_KIN_H
#define CD_KIN_H


int cd_kin_quat_identity(double quat[4]);
int cd_kin_pose_identity(double pose[7]);

int cd_kin_quat_normalize(double quat[4]);
int cd_kin_pose_normalize(double pose[7]);

int cd_kin_quat_flip_closerto(double quat[4], const double target[4]);
int cd_kin_pose_flip_closerto(double pose[7], const double target[7]);


/* composition */
/* qab * qbc = qac
 * aka Hamilton product;
 * it's OK if output is either of the inputs */
int cd_kin_quat_compose(const double quat_ab[4], const double quat_bc[4], double quat_ac[4]);
int cd_kin_pose_compose(const double pose_ab[7], const double pose_bc[7], double pose_ac[7]);

/* Do the same, but for a 3d pos, not a full pose */
int cd_kin_pose_compos(const double pose_ab[7], const double pos_bc[3], double pos_ac[3]);

/* this is rotation, vac = q vbc q* */
int cd_kin_quat_compose_vec(const double quat_ab[4], const double vec_bc[3], double vec_ac[3]);
/* Do the same, but just rotate a 3d vector (vel,acc), not a position vector */
int cd_kin_pose_compose_vec(const double pose_ab[7], const double vec_bc[3], double vec_ac[3]);

/* note conf equals inverse for unit quaternions */
int cd_kin_quat_invert(const double quat_in[4], double quat_out[4]);
int cd_kin_pose_invert(const double pose_in[7], double pose_out[7]);


/* conversion to/from rotation matrices */
int cd_kin_quat_to_R(const double quat[4], double R[3][3]);
int cd_kin_pose_to_H(const double pose[7], double H[4][4], int fill_bottom);
int cd_kin_pose_to_dR(const double pose[7], double d[3], double R[3][3]);

int cd_kin_quat_from_R(double quat[4], double R[3][3]);
int cd_kin_pose_from_H(double pose[7], double H[3][4]);
int cd_kin_pose_from_dR(double pose[7], const double d[3], double R[3][3]);


/* conversion to/from axis-angle */
int cd_kin_quat_to_axisangle(const double quat[4], double axis[3], double *angle);
int cd_kin_quat_from_axisangle(double quat[4], const double axis[3], const double angle);

/* this is the equivalent of cd_kin_quat_compose_vecs above */
int cd_kin_axisangle_rotate(const double axis[3], const double angle,
   const double pos_in[3], double pos_out[3]);

int cd_kin_axisangle_to_R(const double axis[3], const double angle, double R[3][3]);


/* conversion to/from yaw-pitch-roll airplane euler angles */
int cd_kin_quat_to_ypr(const double quat[4], double ypr[3]);
int cd_kin_pose_to_xyzypr(const double pose[7], double xyzypr[6]);

int cd_kin_quat_to_ypr_J(const double quat[4], double J[3][4]);
int cd_kin_pose_to_xyzypr_J(const double pose[7], double J[6][7]);

int cd_kin_quat_from_ypr(double quat[4], const double ypr[3]);
int cd_kin_pose_from_xyzypr(double pose[7], const double xyzypr[6]);


/* convert between pose and pos+quat */
int cd_kin_pose_to_pos_quat(const double pose[7], double pos[3], double quat[4]);
int cd_kin_pose_from_pos_quat(double pose[7], const double pos[3], const double quat[4]);


/* get an arbitrary pose from specification */
int cd_kin_pose_from_op(double pose[7], const double from[3], const double to[3], double * const lenp);
int cd_kin_pose_from_op_diff(double pose[7], const double from[3], const double to_diff[3], double * const lenp);

#endif /* CD_KIN_H */
