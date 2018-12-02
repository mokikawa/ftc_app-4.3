/*
Beginning Comments
Class Description
Mark Okikawa 2018
*/

// Declare package
package org.firstinspires.ftc.teamcode;

// Import statements if any

// File Class
public class  BaseClass {
	
	
// Declare datatypes
	public int myNum;
	public string myName;
	
// Create constructor
	public BaseClass() {
		myNum = 5;			// Set initial myNum to equal 5
		myName = "Mark";	// Set initial myName to equal "Mark"
	}

// 
	public BaseClass (int newNum) {
		myNum = newNum;
		myName = "Mark";
	}

// Create changeNum method
	public void changeNum(int changeBy) {
		myNum += changeBy;
	}
	
// Create changeName method
	public void changeName(String newName) {
		myName = newName;
	}
}