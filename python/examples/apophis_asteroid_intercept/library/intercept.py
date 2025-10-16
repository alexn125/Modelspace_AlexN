###############################################################################
# Copyright (c) ATTX LLC 2024. All Rights Reserved.
#
# This software and associated documentation (the "Software") are the 
# proprietary and confidential information of ATTX, LLC. The Software is 
# furnished under a license agreement between ATTX and the user organization 
# and may be used or copied only in accordance with the terms of the agreement.
# Refer to 'license/attx_license.adoc' for standard license terms.
#
# EXPORT CONTROL NOTICE: THIS SOFTWARE MAY INCLUDE CONTENT CONTROLLED UNDER THE
# INTERNATIONAL TRAFFIC IN ARMS REGULATIONS (ITAR) OR THE EXPORT ADMINISTRATION 
# REGULATIONS (EAR99). No part of the Software may be used, reproduced, or 
# transmitted in any form or by any means, for any purpose, without the express 
# written permission of ATTX, LLC.
###############################################################################
"""
Intercept Function File

Author: Alex Reynolds 
"""
from modelspace.ModelSpacePy import CartesianVector3, Body
from modelspace.LambertTargetTask import LambertTargetTask
from modelspace.Spacecraft import Spacecraft

def identifyIntercept(exc, sc, planet, tof):
    """
    Function to calculate a lambert target of a position 
    
    Params
    ------
    
    
    Return
    ------
    
    """
    # Configure our iterative lambert task 
    lambert = LambertTargetTask()
    lambert.params.mu(sc.gravityModel().params.mu())
    lambert.inputs.time_to_intercept(tof)
    lambert.inputs.trigger(True)
    lambert.inputs.chaser_pos_inertial(sc.body().pos_f_p__p())
    lambert.inputs.chaser_vel_inertial(sc.body().vel_f_p__p())
    lambert.inputs.target_pos_inertial(planet.outputs.inertial_frame().pos_f_p__p())
    lambert.inputs.target_vel_inertial(planet.outputs.inertial_frame().vel_f_p__p())
    
    # Run our lambert solver a couple times to converge on solution
    lambert.startup()
    lambert.step()
    
    # And program a burn from our output
    return lambert.outputs.delta_v(), lambert.outputs.dv_intercept()