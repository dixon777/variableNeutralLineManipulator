import numpy as np
import math as cal

from snake_joint_2d.force_components import *
from snake_joint_2d.displacement_components import *
from snake_joint_2d.helper_functions import *

def defineEndPieceIteractiveFunc(tensionLeftBeforeCableGuide, tensionRightBeforeCableGuide, curvatureAngle, curvatureRadius, frictionCoefficient, length):
    kl_r = endPieceKnobDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isLeft=True)
    kr_r = endPieceKnobDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isLeft=False)
    cgbl_r = cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=False, isLeft=True)
    cgbr_r = cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=False, isLeft=False)
    
    def ___computeJointBendingAngleGuess(jointBendingAngle, returnResultNormalFrictionOnly=True):
        tensionLeftInCableGuide = tensionHoldToLoad(tensionLeftBeforeCableGuide,frictionCoefficient, jointBendingAngle)
        tensionRightInCableGuide = tensionHoldToLoad(tensionRightBeforeCableGuide,frictionCoefficient, jointBendingAngle)
        
        kl_f = endPieceTensionComponent(tensionLeftInCableGuide)
        kr_f = endPieceTensionComponent(tensionRightInCableGuide)
        cgbl_f = cableBendingReactionComponent(tensionLeftInCableGuide, tensionLeftBeforeCableGuide, False, jointBendingAngle)
        cgbr_f = cableBendingReactionComponent(tensionRightInCableGuide, tensionRightBeforeCableGuide, False, jointBendingAngle)
        sum_f = kl_f + kr_f + cgbl_f + cgbr_f
        Nb = -sum_f[0]*cal.sin(jointBendingAngle/2) - sum_f[1]*cal.cos(jointBendingAngle/2)
        Frb = sum_f[0]*cal.cos(jointBendingAngle/2) - sum_f[1]*cal.sin(jointBendingAngle/2)

        Nb_f = contactingNormalForceComponent( jointBendingAngle=jointBendingAngle, isTop=False, magnitude=Nb)
        Frb_f = contactingFrictionComponent(jointBendingAngle=jointBendingAngle, isTop=False, magnitude=Frb)
        contactingb_r = contactingDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, jointBendingAngle=jointBendingAngle,isTop= False)

        result = crossMul2d(kl_r, kl_f)+crossMul2d(kr_r, kr_f)+crossMul2d(cgbl_r, cgbl_f)+crossMul2d(cgbr_r, cgbr_f)+crossMul2d(contactingb_r, Nb_f)+crossMul2d(contactingb_r, Frb_f)
        return (result, jointBendingAngle, Nb, Frb) if returnResultNormalFrictionOnly else (result, {
                "jointBendingAngle": jointBendingAngle,
                "kl": (kl_f, kl_r),
                "kr": (kr_f, kr_r),
                "cgbl": (cgbl_f, cgbl_r),
                "cgbr": (cgbr_f, cgbr_r),
                "Nb": (Nb_f, contactingb_r),
                "Frb": (Frb_f, contactingb_r),
                "result": result
            })
    
    return ___computeJointBendingAngleGuess

def defineIntermediatePieceIteractiveFunc(tensionLeftAfterCableGuide, tensionRightAfterCableGuide, curvatureAngle, curvatureRadius, frictionCoefficient, length, distalJointBendingAngle, distalNormalMagnitude, distalFrictionMagnitude):
    cgtl_r = cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=True, isLeft=True)
    cgtr_r = cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=True, isLeft=False)
    cgbl_r = cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=False, isLeft=True)
    cgbr_r = cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=False, isLeft=False)

    tensionLeftInCableGuide = tensionLoadToHold(tensionLeftAfterCableGuide, frictionCoefficient, distalJointBendingAngle/2)
    tensionRightInCableGuide = tensionLoadToHold(tensionRightAfterCableGuide, frictionCoefficient, distalJointBendingAngle/2)
    contactingt_r = contactingDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, jointBendingAngle=distalJointBendingAngle,isTop= True)

    cgtl_f = cableBendingReactionComponent(tensionLoad=tensionLeftAfterCableGuide, tensionHold=tensionLeftInCableGuide, isTop=True, jointBendingAngle=distalJointBendingAngle)
    cgtr_f = cableBendingReactionComponent(tensionLoad=tensionRightAfterCableGuide, tensionHold=tensionRightInCableGuide, isTop=True, jointBendingAngle=distalJointBendingAngle)
    Nt_f = contactingNormalForceComponent(jointBendingAngle=distalJointBendingAngle, isTop=True,magnitude=distalNormalMagnitude)
    Frt_f = contactingFrictionComponent(jointBendingAngle=distalJointBendingAngle, isTop=True,magnitude=distalFrictionMagnitude)
       
    def ___computeJointBendingAngleGuess(jointBendingAngle, returnResultNormalFrictionOnly=True):
        cgbl_f = cableBendingReactionComponent(tensionLoad=tensionLeftInCableGuide, tensionHold=tensionLoadToHold(tensionLeftInCableGuide, frictionCoefficient, jointBendingAngle/2), isTop=False, jointBendingAngle=jointBendingAngle)
        cgbr_f = cableBendingReactionComponent(tensionLoad=tensionRightInCableGuide, tensionHold=tensionLoadToHold(tensionRightInCableGuide, frictionCoefficient, jointBendingAngle/2), isTop=False, jointBendingAngle=jointBendingAngle)

        sum_f = cgtl_f + cgtr_f + cgbl_f + cgbr_f + Nt_f + Frt_f
        Nb = -sum_f[0]*cal.sin(jointBendingAngle/2) - sum_f[1]*cal.cos(jointBendingAngle/2)
        Frb = sum_f[0]*cal.cos(jointBendingAngle/2) - sum_f[1]*cal.sin(jointBendingAngle/2)

        Nb_f = contactingNormalForceComponent(jointBendingAngle=jointBendingAngle, isTop=False, magnitude=Nb)
        Frb_f = contactingFrictionComponent(jointBendingAngle=jointBendingAngle, isTop=False, magnitude=Frb)
        contactingb_r = contactingDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, jointBendingAngle=jointBendingAngle,isTop= False)

        result = crossMul2d(cgtl_r, cgtl_f)+crossMul2d(cgtr_r, cgtr_f)+crossMul2d(cgbl_r, cgbl_f)+crossMul2d(cgbr_r, cgbl_f)+crossMul2d(contactingt_r, Nt_f)+crossMul2d(contactingt_r, Frt_f)+crossMul2d(contactingb_r, Nb_f)+crossMul2d(contactingb_r, Frb_f)
        
        return (result, jointBendingAngle, Nb, Frb) if returnResultNormalFrictionOnly else (result, {          
                "jointBendingAngle": jointBendingAngle,      
                "cgtl": (cgtl_f, cgtl_r),
                "cgtr": (cgtr_f, cgtr_r),
                "cgbl": (cgbl_f, cgbl_r),
                "cgbr": (cgbr_f, cgbr_r),
                "Nt": (Nt_f, contactingt_r),
                "Frt": (Frt_f, contactingt_r),
                "Nb": (Nb_f, contactingb_r),
                "Frb": (Frb_f, contactingb_r),
                "result": result
            })

    return ___computeJointBendingAngleGuess
"""
Note: The commented computeIterator() is the STRICTLY close loop iteration
Joint angles used for calculation are updated at the end of each loop

Must use joint angles difference between iterations as the determining factor of convergence because the expected tension values
are equal to the evaluated ones from the first loop if the joint bending angles are all zeros, regardless of 
the initial tension values.

However, there is still sizable difference between finalized tension values and the expected ones even if the joint bending
angles converges, and thus new approach is proposed below.
"""      
# def computeIterator(tensionLeft, tensionRight, curvatureRadius, curvatureAngle, numJoints, length, frictionCoefficient, initJointBendingAngles, tensionThreshold=0.001, integralMultiplier=0.1):
#     jointBendingAngles = [i for i in initJointBendingAngles]
#     newJointBendingAngles = [i for i in jointBendingAngles]
#     computedTensionLeft, computedTensionRight = tensionLeft, tensionRight
    
#     indexBigIte = 0
#     while True:
#         # Compute end tensions
#         tensionLeftBeforeEndPieceCableGuide = tensionHoldToLoad(computedTensionLeft, frictionCoefficient, sum(jointBendingAngles[0:-1]))
#         tensionRightBeforeEndPieceCableGuide = tensionHoldToLoad(computedTensionRight, frictionCoefficient, sum(jointBendingAngles[0:-1]))
        
#         # Evalute end piece
#         for (indexSmallIte, res) in enumerate(binarySearchIterator(defineEndPieceIteractiveFunc(tensionLeftBeforeCableGuide=tensionLeftBeforeEndPieceCableGuide, tensionRightBeforeCableGuide=tensionRightBeforeEndPieceCableGuide, curvatureAngle=curvatureAngle, curvatureRadius=curvatureRadius, frictionCoefficient=frictionCoefficient, length=length), lowerBound=-curvatureAngle/2, upperBound=curvatureAngle/2, returnResultNormalFrictionOnly=False)):
#             newJointBendingAngles[-1] = res["jointBendingAngle"]
#             lastN = sum(res["Nb"][0])
#             lastFr = sum(res["Frb"][0])
#             yield indexBigIte, numJoints-1, indexSmallIte, res
            
#         # Evaluate from second end piece to the first piece
#         tensionLeftAfterCableGuide = tensionLeftBeforeEndPieceCableGuide
#         tensionRightAfterCableGuide = tensionRightBeforeEndPieceCableGuide
#         for i in range(numJoints-2, -1, -1):
#             for (indexSmallIte, res) in enumerate(binarySearchIterator(defineIntermediatePieceIteractiveFunc(tensionLeftAfterCableGuide=tensionLeftAfterCableGuide, tensionRightAfterCableGuide=tensionRightAfterCableGuide, curvatureAngle=curvatureAngle, curvatureRadius=curvatureRadius, frictionCoefficient=frictionCoefficient, length=length, distalJointBendingAngle= jointBendingAngles[i+1], distalNormalMagnitude= lastN, distalFrictionMagnitude= lastFr), lowerBound=-curvatureAngle/2, upperBound=curvatureAngle/2, returnResultNormalFrictionOnly=False)):
#                 newJointBendingAngles[i] = res["jointBendingAngle"]
#                 lastN = sum(res["Nb"][0])
#                 lastFr = sum(res["Frb"][0])
#                 yield indexBigIte, i, indexSmallIte, res
#             tensionLeftAfterCableGuide =  tensionLoadToHold(tensionLeftAfterCableGuide, frictionCoefficient, sum(jointBendingAngles[i:i+2])/2)
#             tensionRightAfterCableGuide = tensionLoadToHold(tensionRightAfterCableGuide, frictionCoefficient,  sum(jointBendingAngles[i:i+2])/2)

#         tensionLeftInBase = tensionLoadToHold(tensionLeftAfterCableGuide, frictionCoefficient, jointBendingAngles[0]/2)
#         tensionRightInBase = tensionLoadToHold(tensionRightAfterCableGuide, frictionCoefficient, jointBendingAngles[0]/2)

#         if allWithin(jointBendingAngles,newJointBendingAngles, threshold=0.00001):
#             return jointBendingAngles
        
#         jointBendingAngles = [i for i in newJointBendingAngles]
#         indexBigIte += 1

"""
Note: The computeIterator() below is the ONLINE close loop iteration
Joint angles used for calculation are updated during the iterations. The more proximal joint's static equilibrium is affected by
the latest evaluation of all distal joint's bending angles

As the evaluated tensions are always (have to be verified) different than the expected values. Only tensions are required for determining
convergence

However, there is still sizable difference between finalized tension values and the expected ones even if the joint bending
angles converges. In conclusion, for either batched or online approach, desired results cannot be obtained. Therefore, 
a integral error compensation approach is utilized (similar to the integral controller)
"""        
def computeIterator(tensionLeft, tensionRight, curvatureRadius, curvatureAngle, numJoints, length, frictionCoefficient, initJointBendingAngles, tensionThreshold=0.001, integralMultiplier=0.3):
    jointBendingAngles = [i for i in initJointBendingAngles]
    computedTensionLeft, computedTensionRight = tensionLeft, tensionRight
    
    indexBigIte = 0
    while True:
        # Compute end tensions
        tensionLeftBeforeEndPieceCableGuide = tensionHoldToLoad(computedTensionLeft, frictionCoefficient, sum(jointBendingAngles[0:-1]))
        tensionRightBeforeEndPieceCableGuide = tensionHoldToLoad(computedTensionRight, frictionCoefficient, sum(jointBendingAngles[0:-1]))
        
        # Evalute end piece
        # returnResultNormalFrictionOnly: pass down to defineEndPieceIteractiveFunc()
        for (indexSmallIte, res) in enumerate(binarySearchIterator(defineEndPieceIteractiveFunc(tensionLeftBeforeCableGuide=tensionLeftBeforeEndPieceCableGuide, tensionRightBeforeCableGuide=tensionRightBeforeEndPieceCableGuide, curvatureAngle=curvatureAngle, curvatureRadius=curvatureRadius, frictionCoefficient=frictionCoefficient, length=length), lowerBound=-curvatureAngle/2, upperBound=curvatureAngle/2, returnResultNormalFrictionOnly=False)):
            jointBendingAngles[-1] = res["jointBendingAngle"]
            lastN = sum(res["Nb"][0])
            lastFr = sum(res["Frb"][0])
            yield indexBigIte, numJoints-1, indexSmallIte, res
            
        # Evaluate from second end piece to the first piece
        # returnResultNormalFrictionOnly: pass down to defineEndPieceIteractiveFunc()
        tensionLeftAfterCableGuide = tensionLeftBeforeEndPieceCableGuide
        tensionRightAfterCableGuide = tensionRightBeforeEndPieceCableGuide
        for i in range(numJoints-2, -1, -1):
            for (indexSmallIte, res) in enumerate(binarySearchIterator(defineIntermediatePieceIteractiveFunc(tensionLeftAfterCableGuide=tensionLeftAfterCableGuide, tensionRightAfterCableGuide=tensionRightAfterCableGuide, curvatureAngle=curvatureAngle, curvatureRadius=curvatureRadius, frictionCoefficient=frictionCoefficient, length=length, distalJointBendingAngle= jointBendingAngles[i+1], distalNormalMagnitude= lastN, distalFrictionMagnitude= lastFr), lowerBound=-curvatureAngle/2, upperBound=curvatureAngle/2, returnResultNormalFrictionOnly=False)):
                jointBendingAngles[i] = res["jointBendingAngle"]
                lastN = sum(res["Nb"][0])
                lastFr = sum(res["Frb"][0])
                yield indexBigIte, i, indexSmallIte, res
            tensionLeftAfterCableGuide =  tensionLoadToHold(tensionLeftAfterCableGuide, frictionCoefficient, sum(jointBendingAngles[i:i+2])/2)
            tensionRightAfterCableGuide = tensionLoadToHold(tensionRightAfterCableGuide, frictionCoefficient,  sum(jointBendingAngles[i:i+2])/2)
            tensionLeftInBase = tensionLoadToHold(tensionLeftAfterCableGuide, frictionCoefficient, jointBendingAngles[0]/2)
            tensionRightInBase = tensionLoadToHold(tensionRightAfterCableGuide, frictionCoefficient, jointBendingAngles[0]/2)

        leftError = tensionLeftInBase - tensionLeft
        rightError = tensionRightInBase - tensionRight
        if abs(leftError) < tensionThreshold and abs(rightError) < tensionThreshold:
            return jointBendingAngles
        
        # Psuedo integral factor to eliminate steady state error
        computedTensionLeft -= integralMultiplier*leftError
        computedTensionRight -= integralMultiplier*rightError
        
        indexBigIte += 1
    


# Need fix on output
def compute(tensionLeft, tensionRight, curvatureRadius, curvatureAngle, numJoints, length, frictionCoefficient, initJointBendingAngles, tensionThreshold=0.001):
    jointBendingAngles = [i for i in initJointBendingAngles]
    computedTensionLeft, computedTensionRight = tensionLeft, tensionRight
    while True:
        # Compute end tensions
        tensionLeftBeforeEndPieceCableGuide = tensionHoldToLoad(computedTensionLeft, frictionCoefficient, sum(jointBendingAngles[0:-1]))
        tensionRightBeforeEndPieceCableGuide = tensionHoldToLoad(computedTensionRight, frictionCoefficient, sum(jointBendingAngles[0:-1]))
        
        # Evalute end piece
        # lastN, lastFr, jointBendingAngles[-1] = computeEndPieceAngleIteratively(tensionLeftBeforeCableGuide=tensionLeftBeforeEndPieceCableGuide, tensionRightBeforeCableGuide=tensionRightBeforeEndPieceCableGuide, curvatureAngle=curvatureAngle, curvatureRadius=curvatureRadius, frictionCoefficient=frictionCoefficient, length=length)
        _, jointBendingAngles[-1],lastN, lastFr = binarySearchCompute(defineEndPieceIteractiveFunc(tensionLeftBeforeCableGuide=tensionLeftBeforeEndPieceCableGuide, tensionRightBeforeCableGuide=tensionRightBeforeEndPieceCableGuide, curvatureAngle=curvatureAngle, curvatureRadius=curvatureRadius, frictionCoefficient=frictionCoefficient, length=length), lowerBound=-curvatureAngle/2, upperBound=curvatureAngle/2)

        # Evaluate from second end piece to the first piece
        tensionLeftAfterCableGuide = tensionLeftBeforeEndPieceCableGuide
        tensionRightAfterCableGuide = tensionRightBeforeEndPieceCableGuide
        for i in range(numJoints-2, -1, -1):
            
            # lastN, lastFr, jointBendingAngles[i] = computeIntermediatePieceAngleIteratively(tensionLeftAfterCableGuide=tensionLeftAfterCableGuide, tensionRightAfterCableGuide=tensionRightAfterCableGuide, curvatureAngle=curvatureAngle, curvatureRadius=curvatureRadius, frictionCoefficient=frictionCoefficient, length=length, distalJointBendingAngle= jointBendingAngles[i+1], distalNormalMagnitude= lastN, distalFrictionMagnitude= lastFr)
            _, jointBendingAngles[i],lastN, lastFr = binarySearchCompute(defineIntermediatePieceIteractiveFunc(tensionLeftAfterCableGuide=tensionLeftAfterCableGuide, tensionRightAfterCableGuide=tensionRightAfterCableGuide, curvatureAngle=curvatureAngle, curvatureRadius=curvatureRadius, frictionCoefficient=frictionCoefficient, length=length, distalJointBendingAngle= jointBendingAngles[i+1], distalNormalMagnitude= lastN, distalFrictionMagnitude= lastFr), lowerBound=-curvatureAngle/2, upperBound=curvatureAngle/2)
            tensionLeftAfterCableGuide =  tensionLoadToHold(tensionLeftAfterCableGuide, frictionCoefficient, sum(jointBendingAngles[i:i+2])/2)
            tensionRightAfterCableGuide = tensionLoadToHold(tensionRightAfterCableGuide, frictionCoefficient,  sum(jointBendingAngles[i:i+2])/2)
   
    
        leftError = tensionLeftInBase - tensionLeft
        rightError = tensionRightInBase - tensionRight
        if abs(leftError) < tensionThreshold and abs(rightError) < tensionThreshold:
            return jointBendingAngles
        
        # Psuedo integral factor to eliminate steady state error
        computedTensionLeft -= integralMultiplier*leftError
        computedTensionRight -= integralMultiplier*rightError
            
            
    return jointBendingAngles


