package org.firstinspires.ftc.teamcode;

public class  BaseClass {
	
	public int myNum;
	public string myName;
	
	public BaseClass() {
		myNum = 5;
		myName = "Mark";
	}
	
	public BaseClass (in newNum) {
		myNum = newNum;
		myName = "Mark";
	}
	
	public void changeNum(int changeBy) {
		myNum += changeBy;
	}
	
	public void changeName(String newName) {
		myName = newName;
	}
}