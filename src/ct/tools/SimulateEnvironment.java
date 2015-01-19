package ct.tools;

import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.environment.Environment;

/**
 * @author James MacGlashan.
 */
public class SimulateEnvironment extends Environment{

	protected Domain domain;
	protected RewardFunction rf;
	protected TerminalFunction tf;
	protected int actionBlockTime = 0;

	double lastReward = 0.;

	public SimulateEnvironment(Domain domain, RewardFunction rf, TerminalFunction tf){
		this.domain = domain;
		this.rf = rf;
		this.tf = tf;
	}

	public void setActionBlockTime(int ms){
		this.actionBlockTime = ms;
	}

	@Override
	public State executeAction(String aname, String[] params) {

		Action action = this.domain.getAction(aname);
		State nextState = action.performAction(this.curState, params);
		this.lastReward = this.rf.reward(this.curState, new GroundedAction(action, params), nextState);

		if(this.actionBlockTime > 0) {
			try {
				Thread.sleep(this.actionBlockTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		this.curState = nextState;

		return this.curState;
	}

	@Override
	public double getLastReward() {
		return this.lastReward;
	}

	@Override
	public boolean curStateIsTerminal() {
		return this.tf.isTerminal(this.curState);
	}
}
