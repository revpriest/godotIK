using Godot;
using System;

public partial class SinMovever : Node3D
{
	Node3D parent = null;
	Vector3 initPos = Vector3.Zero;
	float t = 0;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		parent = (Node3D)GetParent();
		initPos = parent.Position;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{	
		t+=(float)delta * 3;
		parent.Position = initPos + new Vector3(Mathf.Cos(t),Mathf.Sin(t/3.3215f),Mathf.Sin(t))*0.04f;
	}
}
