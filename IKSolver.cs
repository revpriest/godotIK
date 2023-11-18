using Godot;
using System;

/**
* Attach to target, or anywhere really.
* 
* Set the TargetNode and bone names etc.
* Probably you'd just create this node with a script,
* and call it's setup function, but you can fill in
* all the details in editor.
*
* Target Node = Where the rig should point
* manget Node = Where the elbow should point if it can (Null for none)
* Look At = Null for keeping bone rotation, a node to look at otherwise
* initBoneNames = strings for root, middle, end bones that will be manipupated.
* 
*
* Mostly from https://github.com/GainAw/GodotCSharpProjectTests/
* Updated to work with Godot4, and some tweeks
* for "lookAt" and tweeks to bone rest position
* by pre@dalliance.net
*/
public partial class IKSolver :Node3D{
	//References to the target Nodes
	[Export] private Skeleton3D targetSkeleton;
	//The node that the IK will point at
	[Export] private Node3D targetNode;
	//Magnet position in respect to the Target Skeleton
	[Export] public Node3D magnetNode=null;	
	//A final node we may rotate the final bone to look at
	[Export] public Node3D lookAt = null;
	//The Ids of the bone chain
	[Export] private int[] boneIds = new int[3];
	[Export] private string[] initBoneNames = null;
	//A way to exit issues in the source skeleton, backwards bones etc.
	[Export] public Vector3 boneRestRotset = Vector3.Zero;

	//Base transforms of the bones in the bone chain
	private Transform3D[] boneBasePose = new Transform3D[3];

	//Individual lengths of the bones
	private float[] boneLengths = new float[2];

	//Floats to tell whether the target is out of the bones range either under or over
	private float _maxChainLength;
	private float _minChainLength;

	//Bool if the ik system is running or not
	[Export] internal bool _isRunning = false;


	/**
	* Set Bone IDs, usually will be called immediately
	* upon creation and so before even _Ready.
	* Base first, Tip last.
	*
	* Use this if you create the node by a script, to
	* set it's values.
	*/
	public void setup(Skeleton3D skel, string[] boneNames, Node3D Target){
		this.targetSkeleton = skel;
		this.targetNode = Target;
		setBonesFromNames(boneNames);
	}

	public void setBonesFromNames(string[] boneNames){
		boneIds = new int[boneNames.Length];
		for(int i=0;i<boneNames.Length;i++){
			boneIds[i] = targetSkeleton.FindBone(boneNames[i]);
		}
	}

	/**
	* Init
	*/
	public override void _Ready() {
		//If there is an init-names list then use it
		if((initBoneNames!=null)&&(initBoneNames.Length>2)){
			setBonesFromNames(initBoneNames);
		}

		//Calculates the lengths of the bones
		boneLengths[0] = targetSkeleton.GetBoneGlobalPose(boneIds[0]).Origin.DistanceTo(targetSkeleton.GetBoneGlobalPose(boneIds[1]).Origin);
		boneLengths[1] = targetSkeleton.GetBoneGlobalPose(boneIds[1]).Origin.DistanceTo(targetSkeleton.GetBoneGlobalPose(boneIds[2]).Origin);

		//Calculates the Out of range variables
		_maxChainLength = boneLengths[0];
		_maxChainLength += boneLengths[1];
		_maxChainLength *= _maxChainLength;
		_minChainLength = Mathf.Abs(boneLengths[0] - boneLengths[1]);
		_minChainLength *= _minChainLength;

		//Sets the base transforms to what the skeleton starts them as
		boneBasePose[0] = targetSkeleton.GetBonePose(boneIds[0]);
		boneBasePose[1] = targetSkeleton.GetBonePose(boneIds[1]);
		boneBasePose[2] = targetSkeleton.GetBonePose(boneIds[2]);

		//Start
		Start();
	}

	
	/**
	* Process update
	*/
	public override void _Process(double delta) {
		solveIk();
	}


	/**
	* Start
	*/
	public void Start() {
		_isRunning = true;
	}

	/**
	* Stop
	*/
	public void Stop(){
		_isRunning = false;
	}


	/**
	* Do a solve
	*/
	private void solveIk() {
		if (_isRunning) {
			//Makes the Targetnode play nice with the skeleton by making it's values local
			Transform3D targetTransform = new Transform3D(new Basis(targetNode.GlobalTransform.Basis.GetRotationQuaternion() * targetSkeleton.GlobalTransform.Basis.GetRotationQuaternion()), 
														  targetSkeleton.ToLocal(targetNode.GlobalTransform.Origin));
			Transform3D tt = new Transform3D();

			//Resets the bones to the base bones in case there is a bug so that the solver doesn't stay broken
			
			targetSkeleton.SetBonePoseRotation(boneIds[0], boneBasePose[0].Basis.GetRotationQuaternion());
			targetSkeleton.SetBonePoseRotation(boneIds[1], boneBasePose[1].Basis.GetRotationQuaternion());
			targetSkeleton.SetBonePoseRotation(boneIds[2], boneBasePose[2].Basis.GetRotationQuaternion());

			//Float to get the distance to the target from the root bone Global pose origin
			float targetDistanceSquared = targetTransform.Origin.DistanceSquaredTo(LocalPoseToGlobalPose(boneIds[0], boneBasePose[0]).Origin);

			//Checks whether the bone is in range or not
			if (targetDistanceSquared >= _maxChainLength || targetDistanceSquared <= _minChainLength) {
				//Out of range Solver 
				for (int i = 0; i < 2; i++) {
					//Uses Normal math to point the chain at the target if it is out of range
					Transform3D boneTransform = targetSkeleton.GetBoneGlobalPose(boneIds[i]);
					Vector3 boneNormal   = targetSkeleton.GetBoneGlobalPose(boneIds[i + 1]).Origin - boneTransform.Origin;
					Vector3 targetNormal =                                    targetTransform.Origin - boneTransform.Origin;
					boneTransform.Basis = boneTransform.Basis.Rotated(boneNormal.Cross(targetNormal).Normalized(), boneNormal.AngleTo(targetNormal));
					tt=GlobalPoseToLocalPose(boneIds[i], boneTransform);
					targetSkeleton.SetBonePoseRotation(boneIds[i], tt.Basis.GetRotationQuaternion());
				}
			}else{
				//Solve for Two Bone
			
				//Makes a target vector based on the target and the root bone
				Vector3 targetVector = targetTransform.Origin - targetSkeleton.GetBoneGlobalPose(boneIds[0]).Origin;

				//Variable to hold lengths needed for law of cosigns
				float[,] lengths = new float[2, 3];
				//Target Triangle lengths
				lengths[0, 0] = boneLengths[0];
				lengths[0, 1] = boneLengths[1];
				lengths[0, 2] = targetVector.Length();

				//Current Triangle Lengths
				lengths[1, 0] = boneLengths[0];
				lengths[1, 1] = boneLengths[1];
				lengths[1, 2] = (targetSkeleton.GetBoneGlobalPose(boneIds[2]).Origin - targetSkeleton.GetBoneGlobalPose(boneIds[0]).Origin).Length();

				//Get Bone Vectors
				Vector3[] boneVector = new Vector3[2];
				boneVector[0] = targetSkeleton.GetBoneGlobalPose(boneIds[0]).Origin - targetSkeleton.GetBoneGlobalPose(boneIds[1]).Origin;
				boneVector[1] = targetSkeleton.GetBoneGlobalPose(boneIds[2]).Origin - targetSkeleton.GetBoneGlobalPose(boneIds[1]).Origin;

				//Get angles need for rotation
				float currentBoneAngle = LawOfCosigns(lengths[1, 2], lengths[1, 1], lengths[1, 0]);
				float targetBoneAngle = LawOfCosigns(lengths[0 ,2], lengths[0, 1], lengths[0, 0]);

				//Solve for the inner angle of the elbow bone by subtracting the targetAngle by the elbow's current inner angle
				float angleToRotate = targetBoneAngle - currentBoneAngle;

				//Get elbow axis of upper arm and lower arm
				Vector3 elbowAxis = (boneVector[0].Cross(boneVector[1])).Normalized();

				//Set elbow inner angle of the second bone transform with the elbowAxis
				Transform3D boneTransform = targetSkeleton.GetBoneGlobalPose(boneIds[1]);
				boneTransform.Basis = boneTransform.Basis.Rotated(elbowAxis, angleToRotate);

				//Set Pose of the elbow with the new transform
				tt = GlobalPoseToLocalPose(boneIds[1], boneTransform);
				targetSkeleton.SetBonePoseRotation(boneIds[1], tt.Basis.GetRotationQuaternion());

				//settings up total chain vector (Vector from the root bone to the tip bone)
				Vector3 chainVector = (targetSkeleton.GetBoneGlobalPose(boneIds[2]).Origin - targetSkeleton.GetBoneGlobalPose(boneIds[0]).Origin);

				//Get the root bone transform and rotate so that it aligns with the target Vector
				boneTransform = targetSkeleton.GetBoneGlobalPose(boneIds[0]);
				boneTransform.Basis = boneTransform.Basis.Rotated(chainVector.Cross(targetVector).Normalized(), chainVector.AngleTo(targetVector));

				//Set Shoulder rotation
				tt = GlobalPoseToLocalPose(boneIds[0], boneTransform);
				targetSkeleton.SetBonePoseRotation(boneIds[0], tt.Basis.GetRotationQuaternion());

				if (magnetNode!=null){
					//Solve for magnet
					//Find the arm chain normal and use it for the normal of the plane
					chainVector = targetSkeleton.GetBoneGlobalPose(boneIds[2]).Origin - targetSkeleton.GetBoneGlobalPose(boneIds[0]).Origin;
					Vector3 chainNormal = chainVector.Normalized();
					Plane magnetPlane = new Plane(chainNormal, 0);

					//Project both the magnet and the elbow to said plane and return their positions;
					Vector3 elbowProject  = magnetPlane.Project(targetSkeleton.GetBoneGlobalPose(boneIds[1]).Origin   - targetSkeleton.GetBoneGlobalPose(boneIds[0]).Origin);
					Vector3 magnetProject = magnetPlane.Project( targetSkeleton.GlobalTransform.AffineInverse() * magnetNode.GlobalPosition -  targetSkeleton.GetBoneGlobalPose(boneIds[0]).Origin);

					//Find the signed rotation between the positions
					float rotation = SignedAngle(elbowProject, magnetProject, chainNormal);

					//Rotate and apply bone rotations
					boneTransform = targetSkeleton.GetBoneGlobalPose(boneIds[0]);
					boneTransform.Basis = boneTransform.Basis.Rotated(chainNormal, rotation);
					targetSkeleton.SetBonePoseRotation(boneIds[0], GlobalPoseToLocalPose(boneIds[0], boneTransform).Basis.GetRotationQuaternion());
				}
			}

			//Set EndBone to be target rotation
			Transform3D endBoneTransform = targetSkeleton.GetBoneGlobalRest(boneIds[2]);
			if(lookAt!=null){
				Vector3 pos = getLocalIgnoringScale(targetSkeleton.GetParent<Node3D>(), lookAt.GlobalPosition);
				pos = targetSkeleton.GlobalTransform.Basis.GetRotationQuaternion().Inverse() * pos;
				endBoneTransform = endBoneTransform.LookingAt(pos,targetSkeleton.Transform.Basis.Y);
			}
			endBoneTransform.Basis = new Basis( (targetSkeleton.GlobalTransform.Basis.GetRotationQuaternion().Inverse() * 
												targetNode.GlobalTransform.Basis.GetRotationQuaternion()) *
												endBoneTransform.Basis.GetRotationQuaternion() * Quaternion.FromEuler(boneRestRotset));
			targetSkeleton.SetBonePoseRotation(boneIds[2], GlobalPoseToLocalPose(boneIds[2], endBoneTransform).Basis.GetRotationQuaternion());
		}
	}


	/**
	* Law of cosigns function that returns the angle of a given triangle
	* given the sides (will solve differently depending on what side is in what variable)
	*/
	private float LawOfCosigns(float a, float b, float c){
		return Mathf.Acos(((b * b) + (c * c) - (a * a)) / (2 * b * c));
	}

	/*
	* Unity function SignedAngle to help with magnet
	* Takes the smallest angle between the vectors 
	* and finds which direction the rotaion goes
	*/
	public float SignedAngle(Vector3 from, Vector3 to, Vector3 axis){
		float unsignedAngle = from.AngleTo(to);

		float cross_X = from.Y * to.Z - from.Z * to.Y;
		float cross_Y = from.Z * to.X - from.X * to.Z;
		float cross_Z = from.X * to.Y - from.Y * to.X;

		float sign = Math.Sign(axis.X * cross_X + axis.Y * cross_Y + axis.Z * cross_Z);
		return unsignedAngle * sign;
	}

	/*
	* Translates the Global pose to a Local pose of a given
	* bone id and returns local pose transform
	*/
	public Transform3D GlobalPoseToLocalPose(int boneId, Transform3D boneGlobalPose){
		int boneParent = targetSkeleton.GetBoneParent(boneId);
		if (boneParent >= 0){
			Transform3D conversionTransform = targetSkeleton.GetBoneGlobalPose(boneParent); // Hummm. Why was this here: * _targetSkeleton.GetBoneRest(boneId) * _targetSkeleton.GetBonePose(boneId);
			conversionTransform = conversionTransform.AffineInverse() * boneGlobalPose;
			return conversionTransform;
		}else{
			return boneGlobalPose;
		}
	}

	/*
	* Translates the Local pose to a Global pose of a given bone id 
	* and returns global pose transform
	*/
	public Transform3D LocalPoseToGlobalPose(int boneId, Transform3D boneLocalPose){
		int boneParent = targetSkeleton.GetBoneParent(boneId);
		if (boneParent >= 0){
			Transform3D conversionTransform = targetSkeleton.GetBoneGlobalPose(boneParent) *  targetSkeleton.GetBonePose(boneId);	
			return conversionTransform;
		}else{
			return boneLocalPose;
		}
	}

	    
    /**
    * Another one that it seems weird is ommited from Godot.
    */
    public static Vector3 getLocalIgnoringScale(Node3D obj, Vector3 globalPosition){
        Quaternion invGlobalRotation = obj.Basis.GetRotationQuaternion().Inverse();

        // Calculate the relative position by subtracting global origin and then rotating
        Vector3 relativePosition = globalPosition - obj.GlobalPosition;
        relativePosition = invGlobalRotation * relativePosition;
    
        return relativePosition;
    }   

}


