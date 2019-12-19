// Authors: Abhilesh Borode, Rachel Breshears, Casey Duncan, Emerson Ham, & Mohammah Aun Siddique
// Date: 5/1/2019
// This code tests the learned Q-Table (produced in rlmagni_withOptiPath.cpp) to prove
// that the Magni Robot learned to follow a straight line correctly.

#include <algorithm>
#include <cstdio>
#include<iostream>
#include<memory>
#include <conio.h>
#include <windows.h>
#include <ctime>
#include <string>
#include <vector>
#include <math.h>
# define M_PI           3.14159265358979323846  /* pi */

using namespace std;

double applyAction(int state, int actionNum)
{
	//This function will be replaced for use in the actual robot
	//This function will receive the "power ratio" and set the robot motor speeds to match it.
	int newState;
	newState = round(state - 2 + actionNum / 2 - 2 + (rand() % 3));
	newState = min(max(newState, 0), 7);
	return newState;
}

double isEnd()
{
	//This function will be replaced in the actual robot.
	//It determines when we have reached the end of the experiment
	int result = 0;
	if ((rand() %10) == 1)
	{
		result = 1;
	}
	return result;
}

int main()
{
	//This function contains the full RL algorithm
	//It will need to be expanded to add new features like:
	//MOO error/reward calculation for position, angle, and speed of travel
	//concurrent learning and implementation
	//Maybe a command to set whether the next turn will be learning or testing
	//Reset wait time at the end so we can re-position the robot

	// -----[Setup]-----
	srand(time(NULL));

	vector<double> stateDivisors{ -M_PI, -3 * M_PI / 9, -2 * M_PI / 9, -M_PI / 9, 0, M_PI / 9, 2 * M_PI / 9, 3 * M_PI / 9, M_PI };
	vector<double> actionRatioPercents{ 0, 12.5, 25, 37.5, 50, 62.5, 75, 87.5, 100 };

	int nStates;
	nStates = stateDivisors.size() - 1;

	int nActions;
	nActions = actionRatioPercents.size() ;

	vector<vector<int>> reward{

	{-4, -3, -2, -1, 0, 1, 2, 3, 4 },
	{-3, -2, -1, 0, 1, 2, 3, 4, 3},
	{-2, -1, 0, 1, 2, 3, 4, 3, 2},
	{-1, 0, 1, 2, 3, 4, 3, 2, 1 },
	{0, 1, 2, 3, 4, 3, 2, 1, 0},
	{1, 2, 3, 4, 3, 2, 1, 0, -1},
	{2, 3, 4, 3, 2, 1, 0, -1, -2},
	{3, 4, 3, 2, 1, 0, -1, -2, -4 } };

	int currentState;
	int newState;
	double discount = 0.7;			//Higher means more influence from how it sets up future movements
	double learnRate = 0.1;		//Higher is faster
	int maxIterations = 1000;	//Number of times to follow the trajectory
	//vector<double> qVals(8, 9);	// Blank Q value matrix, currentState, action

	vector<vector<double>> qVals(8, vector<double>(9, 0));



	for (int i = 0; i <= maxIterations; i++)
	{
		currentState = rand() % nStates;
	
		while (true)
		{
	
			int randAction = rand() % nActions;

			newState = applyAction(currentState, randAction);

			//Calculate the max q - value from the new state
			int maxQ = 0;
			for (int a = 0; a < nActions; a++)
			{
				int val = qVals[newState][a];
				maxQ = max(maxQ,val );
			}

			qVals[currentState][randAction] = (1 - learnRate)*qVals[currentState][randAction] + 
				learnRate*(reward[currentState][randAction] + discount * maxQ);

			if (isEnd() == 1)
			{
				break;
			}

			currentState = newState;

		}
	}
	// find the max value in Q matrix to normalise the Q matrix
	double max2 = 0;
	for (auto& v : qVals)
	{
		double max = *max_element(v.begin(), v.end());
		 max2 = max(max2, max);
	}
	//normalise
	for (auto& v : qVals)
	{
		for (auto& v2 : v)
		{
			v2 = round((v2 / max2)*100);
		}
	}

	

	// Print final Q matrix
	for (const auto &row : qVals)
	{
		for (const auto &s : row) std::cout << s << ' ';
		std::cout << std::endl;
	}
	// find the optimal path
	vector<int> states;
	int state = rand()%nStates;

	states.push_back(state);

	while (isEnd() !=1)
	{
		vector<double> v = qVals[state];
		int maxElementIndex = std::max_element(v.begin(), v.end()) - v.begin();
		int maxElement = *std::max_element(v.begin(), v.end());
		newState = applyAction(state, maxElementIndex);
		states.push_back(newState);
		state = newState;


	}
	cout << endl;
	cout << endl;
	cout << endl;
	cout << endl;

	// print the optimal path

	for (const auto &row : states)
	{
		std::cout << row << ' ';
		//std::cout << std::endl;
	}

	return 0;
}