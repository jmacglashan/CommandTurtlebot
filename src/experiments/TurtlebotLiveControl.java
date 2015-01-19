package experiments;

import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.common.NullRewardFunction;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.visualizer.Visualizer;
import burlap.ros.AsynchronousRosEnvironment;
import commands.data.TrainingElement;
import commands.model3.mt.Tokenizer;
import commands.model3.weaklysupervisedinterface.MTWeaklySupervisedModel;
import commands.model3.weaklysupervisedinterface.WeaklySupervisedController;
import ct.cmdgui.LiveCommands;
import ct.domain.SokoTurtleBot;
import ct.tools.SimulateEnvironment;
import domain.singleagent.sokoban2.Sokoban2Domain;
import domain.singleagent.sokoban2.Sokoban2Visualizer;
import experiments.sokoban.SokobanControllerConstructor;
import generativemodel.GMQueryResult;

import java.util.List;
import java.util.Map;

/**
 * @author James MacGlashan.
 */
public class TurtlebotLiveControl {

	protected SokobanControllerConstructor constructor;
	protected WeaklySupervisedController controller;
	protected List<TrainingElement> sourceDataset;


	public TurtlebotLiveControl(){
		this.constructor = new SokobanControllerConstructor(false, true);
		this.controller = constructor.generateNewController();
		this.addLanguageModel();
	}

	public void addLanguageModel(){
		Tokenizer tokenizer = new Tokenizer(true, true);
		tokenizer.addDelimiter("-");
		MTWeaklySupervisedModel model = new MTWeaklySupervisedModel(controller, tokenizer, 10);

		//set our controller to use the MT model we created
		controller.setLanguageModel(model);

	}

	public void train(String datasetPath){
		//get training data
		this.sourceDataset = constructor.getTrainingDataset(SokobanControllerConstructor.AMTFULLDATASET);

		//instantiate the weakly supervised language model dataset using IRL
		controller.createWeaklySupervisedTrainingDatasetFromTrajectoryDataset(this.sourceDataset);

		//perform learning
		controller.trainLanguageModel();
	}

	public WeaklySupervisedController getController(){
		return this.controller;
	}


	public static class SokoEnv extends AsynchronousRosEnvironment {


		protected int curSeq;

		public SokoEnv(Domain domain, String rosBridgeURI, String rosStateTopic, String rosActionTopic, int actionSleepMS) {
			super(domain, rosBridgeURI, rosStateTopic, rosActionTopic, actionSleepMS);
		}

		public SokoEnv(Domain domain, String rosBridgeURI, String rosStateTopic, String rosActionTopic, int actionSleepMS, int rosBridgeThrottleRate, int rosBridgeQueueLength) {
			super(domain, rosBridgeURI, rosStateTopic, rosActionTopic, actionSleepMS, rosBridgeThrottleRate, rosBridgeQueueLength);
		}

		@Override
		public void receive(Map<String, Object> data, String stringRep) {

			//System.out.println("In receive...");

			Map<String, Object> msg = (Map<String,Object>)data.get("msg");
			Map<String, Object> header = (Map<String, Object>)msg.get("header");
			this.curSeq = (Integer)header.get("seq");
			//System.out.println(seq);
			super.receive(data, stringRep);
		}



		@Override
		protected State onStateReceive(State s) {


			/*
			ObjectInstance room = new ObjectInstance(this.domain.getObjectClass(Sokoban2Domain.CLASSROOM), "macroRoom");
			room.setValue(Sokoban2Domain.ATTTOP, 11);
			room.setValue(Sokoban2Domain.ATTLEFT, 0);
			room.setValue(Sokoban2Domain.ATTBOTTOM, 0);
			room.setValue(Sokoban2Domain.ATTRIGHT, 11);
			room.setValue(Sokoban2Domain.ATTCOLOR, Sokoban2Domain.COLORS[0]);

			s.addObject(room);
			*/

			ObjectInstance room0 = new ObjectInstance(this.domain.getObjectClass(Sokoban2Domain.CLASSROOM), "blueRoom");
			ObjectInstance room1 = new ObjectInstance(this.domain.getObjectClass(Sokoban2Domain.CLASSROOM), "redRoom");
			ObjectInstance room2 = new ObjectInstance(this.domain.getObjectClass(Sokoban2Domain.CLASSROOM), "greenRoom");

			s.addObject(room0);
			s.addObject(room1);
			s.addObject(room2);

			ObjectInstance door0 = new ObjectInstance(this.domain.getObjectClass(Sokoban2Domain.CLASSDOOR), "door0");
			ObjectInstance door1 = new ObjectInstance(this.domain.getObjectClass(Sokoban2Domain.CLASSDOOR), "door1");

			s.addObject(door0);
			s.addObject(door1);

			Sokoban2Domain.setRoom(s, 0, 6, 0, 0, 12, "blue");
			Sokoban2Domain.setRoom(s, 1, 12, 0, 6, 6, "green");
			Sokoban2Domain.setRoom(s, 2, 12, 6, 6, 12, "red");

			Sokoban2Domain.setDoor(s, 0, 6, 2, 6, 4);
			Sokoban2Domain.setDoor(s, 1, 6, 8, 6, 10);


			//System.out.println("Received: " + this.curSeq + this.agentString(s));

			return s;

		}

		@Override
		public State executeAction(String aname, String[] params) {

			//System.out.println("Enter action " + this.agentString(this.curState) + ": " + aname);

			State end = super.executeAction(aname, params);

			//System.out.println("return state: " + this.agentString(end));
			//System.out.println("--");

			return end;
		}


		protected String agentString(State s){
			ObjectInstance agent = s.getFirstObjectOfClass(Sokoban2Domain.CLASSAGENT);
			int ax = agent.getDiscValForAttribute(Sokoban2Domain.ATTX);
			int ay = agent.getDiscValForAttribute(Sokoban2Domain.ATTY);
			String dir = agent.getStringValForAttribute(Sokoban2Domain.ATTDIR).substring(0, 1);
			return "(" + ax + "," + ay + "," + dir + ")";
		}
	}

	public static Environment getSimulatedEnvironment(Domain domain){

		SimulateEnvironment env = new SimulateEnvironment(domain, new NullRewardFunction(), new NullTermination());
		env.setActionBlockTime(700);

		State s = Sokoban2Domain.getCleanState(domain, 3, 2, 1);
		Sokoban2Domain.setRoom(s, 0, 4, 0, 0, 12, "blue");
		Sokoban2Domain.setRoom(s, 1, 8, 0, 4, 6, "green");
		Sokoban2Domain.setRoom(s, 2, 8, 6, 4, 12, "red");

		Sokoban2Domain.setDoor(s, 0, 4, 3, 4, 3);
		Sokoban2Domain.setDoor(s, 1, 4, 9, 4, 9);

		Sokoban2Domain.setBlock(s, 0, 6, 2, Sokoban2Domain.SHAPES[0], "yellow");

		Sokoban2Domain.setAgent(s, 8, 5);
		env.setCurStateTo(s);

		return env;

	}

	public static Environment getROSEnvironment(Domain domain){

		//setup ROS information
		String uri = "ws://chelone:9090";
		String stateTopic = "/burlap_state";
		String actionTopic = "/burlap_action";
		//create environment with 4000ms (4s) action execution time
		final AsynchronousRosEnvironment env = new SokoEnv(domain, uri, stateTopic, actionTopic, 4000, 16, 1);
		env.blockUntilStateReceived();

		return env;

	}

	public static void main(String[] args) {

		SokoTurtleBot stb = new SokoTurtleBot();
		Domain domain = stb.generateDomain();
		//Environment env = getSimulatedEnvironment(domain);
		Environment env = getROSEnvironment(domain);
		Visualizer v = Sokoban2Visualizer.getVisualizer("robotImages");


		TurtlebotLiveControl tlc = new TurtlebotLiveControl();
		//tlc.train(SokobanControllerConstructor.AMTFULLDATASET);

		LiveCommands lc = new LiveCommands(domain, env, v, tlc.getController(), new SokoAStarPlanner());
		lc.addKeyAction("w", new GroundedAction(domain.getAction(SokoTurtleBot.ACTIONFORWARD), ""));
		lc.addKeyAction("d", new GroundedAction(domain.getAction(SokoTurtleBot.ACTIONROTATE), ""));
		lc.addKeyAction("a", new GroundedAction(domain.getAction(SokoTurtleBot.ACTIONROTATECCW), ""));
		lc.beginLiveStream();
		lc.showGUI();

		/*TrainingElement te = tlc.sourceDataset.get(0);
		List<GMQueryResult> queryResults = tlc.controller.getRFDistribution(te.trajectory.getState(0), te.command);

		System.out.println("Distribution for: " + te.command);
		for(GMQueryResult r : queryResults){
			System.out.println(r.probability + ": " + r.getSingleQueryVar().toString());
		}*/


		/*State s = Sokoban2Domain.getCleanState(domain, 1, 0, 0);
		Sokoban2Domain.setAgent(s, 1, 1);
		Sokoban2Domain.setRoom(s, 0, 12, 0, 0, 12, Sokoban2Domain.COLORS[0]);*/










	}






}
