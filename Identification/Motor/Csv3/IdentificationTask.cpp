#include "IdentificationTask.hpp"

LoopTask* create_task(Hardware* hw)
{

    double* amps = new double[13];
    double* freqs = new double[13];
    amps[0] = 5.0;//10
    freqs[0] = 1.0;
    amps[1] = 5.0;
    freqs[1] = 2.0;
    amps[2] = 3.0;
    freqs[2] = 4.0;
    amps[3] = 2.0;
    freqs[3] = 5.0;
    amps[4] = 1.5;
    freqs[4] = 6.0;
	amps[5] = 1.0;
    freqs[5] = 7.0;
	amps[6] = 1.0;
    freqs[6] = 8.0;
	amps[7] = 0.8;
    freqs[7] = 9.0;
	amps[8] = 0.8;
    freqs[8] = 10.0;
	amps[9] = 0.8;
    freqs[9] = 11.0;
	amps[10] = 0.5;
    freqs[10] = 12.0;
	amps[11] = 0.5;
    freqs[11] = 15.0;
    amps[12] = 0.1;
    freqs[12] = 20.0;



//    amps[5] = 15.0;
//    freqs[5] = 1.0;
//    amps[6] = 20.0;
//    freqs[6] = 0.5;
//    amps[7] = 30.0;
//    freqs[7] = 0.5;
//    amps[8] = 40.0;
//    freqs[8] = 0.2;
//    amps[9] = 60.0;
//    freqs[9] = 0.2;

    IdentificationTask* id = new IdentificationTask(amps,freqs,13,hw);
    id->filename = "Identification";

    double* amplitudes = new double[10];

//    amplitudes[0] = -0.7;
//    amplitudes[1] = -0.16;
//    amplitudes[2] = -0.14;
//    amplitudes[3] = -0.8;
//    amplitudes[4] = -0.18;
//    amplitudes[5] = -0.2;
//    amplitudes[6] = -0.3;
//    amplitudes[7] = -0.4;
//    amplitudes[8] = -0.5;
//    amplitudes[9] = -0.6;

	amplitudes[0] = 1;
    amplitudes[1] = -1;
    amplitudes[2] = 0.9;
    amplitudes[3] = -0.9;
    amplitudes[4] = 0.7;
    amplitudes[5] = -0.7;
    amplitudes[6] = 0.6;
    amplitudes[7] = -0.6;
    amplitudes[8] = 0.5;
    amplitudes[9] = -0.5;

//    amplitudes[0] = -0.10;
//    amplitudes[1] = -0.11;
//    amplitudes[2] = -0.12;
//    amplitudes[3] = -0.13;
//    amplitudes[4] = -0.14;
//    amplitudes[5] = -0.15;
//    amplitudes[6] = -0.16;
//    amplitudes[7] = -0.17;
//    amplitudes[8] = -0.18;
//    amplitudes[9] = -0.19;

    FrictionIdentificationTask* f = new FrictionIdentificationTask(amplitudes,10,hw);
    string sf("Identification");
    f->filename = sf;

	//--------------------------------------

//	#define LEN 20
//    double* cur_amps = new double[LEN];
//    double* cur_freqs = new double[LEN];
//
//    for(int i = 0;i<LEN;i++)
//    {
////		cur_freqs[i] = (double)(i+1)*0.5;
////		cur_amps[i] = 1.0;
//
//		cur_freqs[i] = (double)(i+1)*0.5;
//		cur_amps[i] = 4.0;
//    }
//
//    openLoopIdentificationTask* ol = new openLoopIdentificationTask(cur_amps,cur_freqs,LEN,hw);
//    string sol("sinCurrentLogLong");
//    ol->filename = sol;

    return id;
}

IdentificationTask::IdentificationTask(double* amps, double* freqs, int n, Hardware* hw): LoopTask(hw)
{
    amp = amps;
    freq = freqs;
    size = n;
}

int IdentificationTask::_loop()
{
	if (idx >= size) return 0; //all experiments has been done*/

    if (hw->isStandStill(3333))
    {
        hw->resetM();
        hw->resetE();

		if(pt != NULL)
		{
			//pt->dispose();
			//delete pt;
		}

        pt = new SinPositionTask(hw);
        pt->amp = amp[idx]*0.5;
        pt->freq = freq[idx]* 0.5;
        pt->logEnabled = true;
        pt->duration = 7;
        pt->next = this;
//        pt->ctr->KP = 1.0;
        task = pt;
        pt->filename = this->filename;

        cout << "Experiment n." << idx << endl;
        idx++;
    }
    return 1;
}


int FrictionIdentificationTask::_loop()
{
	if (idx >= size) return 0; //all experiments has been done*/

    if (hw->isStandStill(3333))
    {
        hw->resetM();
        hw->resetE();

		/*if(t != NULL)
		{
			//t->dispose();
			//delete t;
		}*/

        ct = new CurrentTask(hw);
		ct->current_ref= amp[idx]*0.2;
        ct->logEnabled = true;
        ct->duration = 7;
        ct->next = this;
        ct->filename = this->filename;
        task = ct;

//        vt = new VelocityTask(hw);
//		vt->velocity_ref= amp[idx]*1;//to tune;
//        vt->logEnabled = true;
//        vt->duration = 7;
//        vt->next = this;
//        vt->filename = this->filename;
//        task = vt;

        cout << "Experiment n." << idx << endl;

        idx++;

    }
//    if (idx > size) return 0; //all experiments has been done*/
    return 1;
}


int openLoopIdentificationTask::_loop()
{
    if (hw->isStandStill(3333))
    {
        hw->resetM();
        hw->resetE();

		/*if(t != NULL)
		{
			//t->dispose();
			//delete t;
		}*/

        t = new SinCurrentTask(hw);
		t->amp = amp[idx];
        t->freq = freq[idx];
        t->logEnabled = true;
        t->duration = 7;
        t->next = this;
        t->filename = this->filename;
        task = t;

        cout << "Experiment n." << idx << endl;

        idx++;

    }
    if (idx > size) return 0; //all experiments has been done*/
    return 1;
}


