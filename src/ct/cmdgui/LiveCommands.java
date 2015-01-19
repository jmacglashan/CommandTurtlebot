package ct.cmdgui;

import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.learning.modellearning.DomainMappedPolicy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.NullRewardFunction;
import burlap.oomdp.singleagent.environment.DomainEnvironmentWrapper;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.visualizer.Visualizer;
import commands.model3.TaskModule;
import commands.model3.TrajectoryModule;
import commands.model3.weaklysupervisedinterface.WeaklySupervisedController;
import ct.domain.SokoTurtleBot;
import ct.tools.SimulateEnvironment;
import domain.singleagent.sokoban2.Sokoban2Domain;
import domain.singleagent.sokoban2.Sokoban2Visualizer;
import generativemodel.GMQueryResult;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.HashMap;
import java.util.List;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Map;

/**
 * @author James MacGlashan.
 */
public class LiveCommands extends JFrame{

	protected Domain planningDomain;
	protected Environment env;
	protected Domain envDomain;
	protected WeaklySupervisedController lcontroller;
	protected PolicyGenerator policyGen;

	protected Visualizer v;

	protected JTextArea commandArea;
	protected JButton executeCommandButton;
	protected JButton stopExecution;

	protected Thread liveStreamThread;
	protected Thread policyThread;
	protected volatile boolean humanTerminateSignal = false;

	protected Map<String, GroundedAction> keyActionMap = new HashMap<String, GroundedAction>();


	protected volatile boolean runLiveStream = false;

	public LiveCommands(Domain planningDomain, Environment env, Visualizer v, WeaklySupervisedController lcontroller, PolicyGenerator policyGenerator){
		this.planningDomain = planningDomain;
		this.env = env;
		DomainEnvironmentWrapper wrapper = new DomainEnvironmentWrapper(this.planningDomain, this.env);
		this.envDomain = wrapper.generateDomain();
		this.v = v;
		this.lcontroller = lcontroller;
		this.policyGen = policyGenerator;
		this.initGUI();
	}


	public void addKeyAction(String key, GroundedAction action){
		this.keyActionMap.put(key, action);
	}


	protected void initGUI(){

		this.v.setPreferredSize(new Dimension(600, 600));
		this.getContentPane().add(this.v, BorderLayout.CENTER);

		Container guiControls = new Container();
		this.getContentPane().add(guiControls, BorderLayout.WEST);
		guiControls.setPreferredSize(new Dimension(400, 600));
		guiControls.setLayout(new GridBagLayout());

		GridBagConstraints c = new GridBagConstraints();
		c.gridx = 0;
		c.gridy = 0;
		c.insets = new Insets(10, 10, 10, 10);

		this.commandArea = new JTextArea(5, 30);
		guiControls.add(this.commandArea, c);

		c.gridy = 1;
		this.executeCommandButton = new JButton("Execute Command");
		this.executeCommandButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				LiveCommands.this.executeCommand();
			}
		});
		guiControls.add(this.executeCommandButton, c);

		c.gridy = 2;
		this.stopExecution = new JButton("Stop Execution");
		this.stopExecution.setEnabled(false);
		this.stopExecution.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				LiveCommands.this.stopExecution.setEnabled(false);
				LiveCommands.this.humanTerminateSignal = true;
			}
		});
		guiControls.add(this.stopExecution, c);


		this.v.addKeyListener(new KeyListener() {
			@Override
			public void keyTyped(KeyEvent e) {
				String strKey = String.valueOf(e.getKeyChar());
				GroundedAction ga = LiveCommands.this.keyActionMap.get(strKey);
				if(ga != null){
					LiveCommands.this.env.executeAction(ga);
				}
			}

			@Override
			public void keyPressed(KeyEvent e) {

			}

			@Override
			public void keyReleased(KeyEvent e) {

			}
		});

	}


	public void showGUI(){
		this.beginLiveStream();
		this.pack();
		this.setVisible(true);
	}

	protected void executeCommand(){
		String command = this.commandArea.getText().trim();
		this.executeCommandButton.setEnabled(false);

		System.out.println("Receiving command: " + command);

		//do command inference, plan, and execute
		List<GMQueryResult> predictions = this.lcontroller.getRFDistribution(this.env.getCurState(), command);
		TaskModule.RFConVariableValue val = (TaskModule.RFConVariableValue)GMQueryResult.maxProb(predictions).getSingleQueryVar();
		TrajectoryModule.ConjunctiveGroundedPropTF tf = new TrajectoryModule.ConjunctiveGroundedPropTF(val.rf);
		final RFTF rftf = new RFTF(val.rf, tf);
		System.out.println("Selecting RF: " + val.toString());
		//Policy plannerPolicy = this.getPlannerPolicy(rftf);
		Policy plannerPolicy = this.policyGen.getPolicy(this.planningDomain, this.env.getCurState(), rftf.rf, rftf.tf,
				new DiscreteStateHashFactory());
		final Policy envPolicy = new DomainMappedPolicy(this.envDomain, plannerPolicy);

		this.policyThread = new Thread(new Runnable() {
			@Override
			public void run() {
				LiveCommands.this.stopExecution.setEnabled(true);
				envPolicy.evaluateBehavior(LiveCommands.this.env.getCurState(), rftf.rf,
						new TermWithHumanIntercept(rftf.tf), 60);
				LiveCommands.this.executeCommandButton.setEnabled(true);
				LiveCommands.this.humanTerminateSignal = false;
				LiveCommands.this.stopExecution.setEnabled(false);
			}
		});

		this.policyThread.start();

	}

	public void beginLiveStream(){


		if(!this.runLiveStream) {

			this.runLiveStream = true;

			if (this.liveStreamThread == null) {
				this.liveStreamThread = new Thread(new Runnable() {
					@Override
					public void run() {
						while (LiveCommands.this.runLiveStream) {
							LiveCommands.this.v.updateState(env.getCurState());
							try {
								Thread.sleep(100);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}
						}
					}
				});

			}

			this.liveStreamThread.start();

		}

	}


	protected Policy getPlannerPolicy(RFTF rftf){

		ValueIteration vi = new ValueIteration(this.planningDomain, rftf.rf, rftf.tf, 0.99, new DiscreteStateHashFactory(),
				0.01, 100);

		vi.planFromState(this.env.getCurState());
		Policy p = new GreedyQPolicy(vi);


		return p;
	}


	public void pauseLiveStream(){
		this.runLiveStream = false;
	}


	public static class RFTF{
		public RewardFunction rf;
		public TerminalFunction tf;

		public RFTF(RewardFunction rf, TerminalFunction tf){
			this.rf = rf;
			this.tf = tf;
		}
	}

	public class TermWithHumanIntercept implements TerminalFunction{

		TerminalFunction sourceTF;

		public TermWithHumanIntercept(TerminalFunction sourceTF){
			this.sourceTF = sourceTF;
		}


		@Override
		public boolean isTerminal(State s) {
			return LiveCommands.this.humanTerminateSignal || this.sourceTF.isTerminal(s);
		}
	}

	public static void main(String[] args) {
		SokoTurtleBot stb = new SokoTurtleBot();
		Domain domain = stb.generateDomain();
		Environment env = new SimulateEnvironment(domain, new NullRewardFunction(), new NullTermination());
		Visualizer v = Sokoban2Visualizer.getVisualizer("robotImages");

		State s = Sokoban2Domain.getCleanState(domain, 1, 0, 0);
		Sokoban2Domain.setAgent(s, 1, 1);
		Sokoban2Domain.setRoom(s, 0, 12, 0, 0, 12, Sokoban2Domain.COLORS[0]);
		env.setCurStateTo(s);


		LiveCommands lc = new LiveCommands(domain, env, v, null, null);
		lc.beginLiveStream();
		lc.showGUI();

	}

}
