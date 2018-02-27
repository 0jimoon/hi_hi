#include "stdafx.h"
#include "KuTracker.h"

using namespace cv;
using namespace std;

size_t KuTrack::NextTrackID=0;
// ---------------------------------------------------------------------------
// Track constructor.
// The track begins from initial point (pt)
// ---------------------------------------------------------------------------
KuTrack::KuTrack(Point2f pt, float dt, float Accel_noise_mag)
{
	track_id=NextTrackID;
	NextTrackID++;
	// Every track have its own Kalman filter,
	// it user for next point position prediction.
	KF = new KuKalman(pt,dt,Accel_noise_mag);
	// Here stored points coordinates, used for next position prediction.
	prediction=pt;
	skipped_frames=0;
	if(NextTrackID > 50){
		NextTrackID = 0;
	}
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
KuTrack::~KuTrack()
{
	// Free resources.
	delete KF;
}

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
KuTracker::KuTracker(float _dt, float _Accel_noise_mag, double _dist_thres, int _maximum_allowed_skipped_frames,int _max_trace_length)
{
	dt=_dt;
	Accel_noise_mag=_Accel_noise_mag;
	dist_thres=_dist_thres;
	maximum_allowed_skipped_frames=_maximum_allowed_skipped_frames;
	max_trace_length=_max_trace_length;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void KuTracker::Update(vector<Point2f>& detections)
{
	// -----------------------------------
	// If there is no tracks yet, then every point begins its own track.
	// -----------------------------------
	if(kutracks.size()==0)
	{
		// If no tracks yet
		for(int i=0;i<detections.size();i++)
		{
			KuTrack* tr=new KuTrack(detections[i],dt,Accel_noise_mag);
			kutracks.push_back(tr);
		}	
	}

	// -----------------------------------
	// «десь треки уже есть в любом случае
	// -----------------------------------
	int N=kutracks.size();		// треки
	int M=detections.size();	// детекты

	// ћатрица рассто¤ний от N-ного трека до M-ного детекта.
	vector< vector<double> > Cost(N,vector<double>(M));
	vector<int> assignment; // назначени¤

	// -----------------------------------
	// “реки уже есть, составим матрицу рассто¤ний
	// -----------------------------------
	double dist;
	for(int i=0;i<kutracks.size();i++)
	{	
		// Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for(int j=0;j<detections.size();j++)
		{
			Point2d diff=(kutracks[i]->prediction-detections[j]);
			dist=sqrtf(diff.x*diff.x+diff.y*diff.y);
			Cost[i][j]=dist;
		}
	}
	// -----------------------------------
	// Solving assignment problem (tracks and predictions of Kalman filter)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost,assignment,AssignmentProblemSolver::optimal);

	// -----------------------------------
	// clean assignment from pairs with large distance
	// -----------------------------------
	// Not assigned tracks
	vector<int> not_assigned_tracks;

	for(int i=0;i<assignment.size();i++)
	{
		if(assignment[i]!=-1)
		{
			if(Cost[i][assignment[i]]>dist_thres)
			{
				assignment[i]=-1;
				// Mark unassigned tracks, and increment skipped frames counter,
				// when skipped frames counter will be larger than threshold, track will be deleted.
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{			
			// If track have no assigned detect, then increment skipped frames counter.
			kutracks[i]->skipped_frames++;
			int a;
			a=kutracks[i]->skipped_frames;

		}

	}

	// ---------------- -------------------
	// If track didn't get detects long time, remove it.
	// -----------------------------------
	for(int i=0;i<kutracks.size();i++)
	{
		if(kutracks[i]->skipped_frames>maximum_allowed_skipped_frames)
		{
			delete kutracks[i];
			kutracks.erase(kutracks.begin()+i);
			assignment.erase(assignment.begin()+i);
			i--;
		}
	}
	// -----------------------------------
	// Search for unassigned detects
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for(int i=0;i<detections.size();i++)
	{
		it=find(assignment.begin(), assignment.end(), i);
		if(it==assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	// -----------------------------------
	// and start new tracks for them.
	// -------------------------- ---------
	if(not_assigned_detections.size()!=0)
	{
		for(int i=0;i<not_assigned_detections.size();i++)
		{
			KuTrack* tr=new KuTrack(detections[not_assigned_detections[i]],dt,Accel_noise_mag);
			kutracks.push_back(tr);
		}	
	}

	// Update Kalman Filters state

	for(int i=0;i<assignment.size();i++)
	{
		// If track updated less than one time, than filter state is not correct.

		kutracks[i]->KF->GetPrediction();

		if(assignment[i]!=-1) // If we have assigned detect, then update using its coordinates,
		{
			kutracks[i]->skipped_frames=0;
			kutracks[i]->prediction=kutracks[i]->KF->Update(detections[assignment[i]],1);
			kutracks[i]->center=detections[assignment[i]];
		}else				  // if not continue using predictions
		{
			kutracks[i]->prediction=kutracks[i]->KF->Update(Point2f(0,0),0);	
		}

		if(kutracks[i]->trace.size()>max_trace_length)
		{
			kutracks[i]->trace.erase(kutracks[i]->trace.begin(),kutracks[i]->trace.end()-max_trace_length);
		}

		kutracks[i]->trace.push_back(kutracks[i]->prediction);
		kutracks[i]->KF->LastResult=kutracks[i]->prediction;
	}

}



// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
KuTracker::~KuTracker(void)
{
	for(int i=0;i<kutracks.size();i++)
	{
		delete kutracks[i];
	}
	kutracks.clear();
}

