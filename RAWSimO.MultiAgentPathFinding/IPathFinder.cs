using RAWSimO.MultiAgentPathFinding.Elements;
using RAWSimO.MultiAgentPathFinding.Physic;
using RAWSimO.MultiAgentPathFinding.Toolbox;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace RAWSimO.MultiAgentPathFinding
{
    /// <summary>
    /// Interface for an optimizer for the Multi Agent Path Finding Problem
    /// translate abouve message to chinese: 多智能体路径规划问题的优化器的接口
    /// </summary>
    public abstract class PathFinder
    {
        /// <summary>
        /// graph
        /// </summary>
        public Graph Graph;

        /// <summary>
        /// The stopwatch
        /// translate abouve message to chinese: 计时器
        /// </summary>
        public Stopwatch Stopwatch;

        /// <summary>
        /// The length of a wait step
        /// translate abouve message to chinese: 等待步长的长度
        /// </summary>
        public double LengthOfAWaitStep = 5.0;

        /// <summary>
        /// The runtime limit per call per agent
        /// translate abouve message to chinese: 每个代理的每次调用的运行时限制
        /// </summary>
        public double RuntimeLimitPerAgent = .05;

        /// <summary>
        /// The runtime limit per call
        /// translate abouve message to chinese: 每次调用的运行时限制
        /// </summary>
        public double RunTimeLimitOverall = 1.0;

        /// <summary>
        /// The seed to use for all randomizers.
        /// translate abouve message to chinese: 用于所有随机器的种子
        /// </summary>
        protected Random Randomizer;

        /// <summary>
        /// The logger to use.
        /// translate abouve message to chinese: 用于所有随机器的种子
        /// </summary>
        internal PathPlanningCommunicator Communicator; // internal means that it is only visible to the current assembly

        /// <summary>
        /// constructor
        /// translate abouve message to chinese: 构造函数
        /// </summary>
        /// <param name="graph">graph</param>
        /// <param name="seed">The seed to use for the randomizer.</param>
        /// <param name="communicator">The logger to use.</param>
        public PathFinder(Graph graph, int seed, PathPlanningCommunicator communicator)
        {
            this.Randomizer = new Random(seed); // seed is used to generate a sequence of numbers that are not random but deterministic, this. means that the randomizer is only visible to the current class
            this.Graph = graph; // graph is used to store the graph
            this.Stopwatch = new Stopwatch(); // stopwatch is used to measure the time
            this.Communicator = communicator; // communicator is used to log the information
        }

        /// <summary>
        /// Find the path for all the agents.
        /// translate abouve message to chinese: 查找所有代理的路径
        /// </summary>
        /// <param name="currentTime">The current time.</param>
        /// <param name="agents">agents</param>
        public abstract void FindPaths(double currentTime, List<Agent> agents); // public means that it is visible to all assemblies
        /// abstract means that the method is not implemented here but in a derived class

        /// <summary>
        /// Finds a sequence of elevators with must be visit to reach the end node.
        /// </summary>
        /// <param name="startNode">The start node.</param>
        /// <param name="endNode">The end node.</param>
        /// <param name="physics">The physics.</param>
        /// <returns>elevator sequence</returns>
        public List<Tuple<object, int, int>> FindElevatorSequence(int startNode, int endNode, Physics physics, out double distance)
        {
            return ElevatorSequenceFinder.FindElevatorSequence(Graph, physics, startNode, endNode, out distance);
        }

    }

    #region Communication with the Instance 

    /// <summary>
    /// Used to wrap a communication functionality.
    /// </summary>
    public class PathPlanningCommunicator
    {
        /// <summary>
        /// A dummy logger that can be used that does nothing.
        /// translate abouve message to chinese: 一个可以什么都不做的虚拟日志记录器
        /// </summary>
        public static readonly PathPlanningCommunicator DUMMY_COMMUNICATOR = new PathPlanningCommunicator(null, null, null, null, null);
        /// <summary>
        /// Creates a new instance of the wrapper.
        /// </summary>
        /// <param name="logSevere">Used to log severe errors. (This is log level 1)</param>
        /// <param name="logDefault">Used to log standard messages of the simulation progress or severe warnings / bug info. (This is log level 2)</param>
        /// <param name="logInfo">Used to log informative messages about things potentially going wrong or the state of the system. (This is log level 3)</param>
        /// <param name="logVerbose">Used to log everything that can be logged. (This is log level 4)</param>
        /// <param name="signalTimeout">Notifies the instance about a timeout.</param>
        public PathPlanningCommunicator(Action<string> logSevere, Action<string> logDefault, Action<string> logInfo, Action<string> logVerbose, Action signalTimeout)
        { _logSevere = logSevere; _logDefault = logDefault; _logInfo = logInfo; _logVerbose = logVerbose; _signalTimeout = signalTimeout; }

        #region Logging

        /// <summary>
        /// Used to log severe errors. (This is log level 1)
        /// </summary>
        private Action<string> _logSevere;
        /// <summary>
        /// Used to log standard messages of the simulation progress or severe warnings / bug info. (This is log level 2)
        /// </summary>
        private Action<string> _logDefault;
        /// <summary>
        /// Used to log informative messages about things potentially going wrong or the state of the system. (This is log level 3)
        /// </summary>
        private Action<string> _logInfo;
        /// <summary>
        /// Used to log everything that can be logged. (This is log level 4)
        /// </summary>
        private Action<string> _logVerbose;
        /// <summary>
        /// Used to log severe errors. (This is log level 1)
        /// </summary>
        /// <param name="message">The message to log.</param>
        internal void LogSevere(string message) { _logSevere?.Invoke(message); }
        /// <summary>
        /// Used to log standard messages of the simulation progress or severe warnings / bug info. (This is log level 2)
        /// </summary>
        /// <param name="message">The message to log.</param>
        internal void LogDefault(string message) { _logDefault?.Invoke(message); }
        /// <summary>
        /// Used to log informative messages about things potentially going wrong or the state of the system. (This is log level 3)
        /// </summary>
        /// <param name="message">The message to log.</param>
        internal void LogInfo(string message) { _logInfo?.Invoke(message); }
        /// <summary>
        /// Used to log everything that can be logged. (This is log level 4)
        /// </summary>
        /// <param name="message">The message to log.</param>
        internal void LogVerbose(string message) { _logVerbose?.Invoke(message); }

        #endregion

        #region Statistics

        /// <summary>
        /// Notifies the instance about a timeout.
        /// </summary>
        private Action _signalTimeout;
        /// <summary>
        /// Notifies the instance about a timeout.
        /// </summary>
        internal void SignalTimeout() { _signalTimeout?.Invoke(); }

        #endregion
    }

    #endregion

    #region Path implementation

    /// <summary>
    /// solution path
    /// translate abouve message to chinese: 解决方案路径
    /// </summary>
    public class Path
    {
        /// <summary>
        /// List of Actions
        /// </summary>
        private LinkedList<Action> _actions = new LinkedList<Action>();

        /// <summary>
        /// All actions of this path.
        /// </summary>
        public IEnumerable<Action> Actions { get { return _actions; } }

        /// <summary>
        /// Gets the next action.
        /// </summary>
        /// <value>
        /// The next action.
        /// </value>
        public Action NextAction { get { return _actions.First.Value; } }

        /// <summary>
        /// Gets the last action.
        /// </summary>
        /// <value>
        /// The last action.
        /// </value>
        public Action LastAction { get { return _actions.Last.Value; } }

        /// <summary>
        /// ID of the next node to turn towards after completing the path, if given. If a value smaller than 0 is given, no preparation will be executed.
        /// </summary>
        public int NextNodeToPrepareFor = -1;

        /// <summary>
        /// Gets the count.
        /// </summary>
        /// <value>
        /// The count.
        /// </value>
        public int Count { get { return _actions.Count; } }

        /// <summary>
        /// Gets a value indicating whether this instance is consistent.
        /// </summary>
        /// <value>
        /// <c>true</c> if this instance is consistent; otherwise, <c>false</c>.
        /// </value>
        public bool IsConsistent
        {
            get
            {
                return _actions.Count == 0 || _actions.Last().StopAtNode;
            }
        }

        /// <summary>
        /// Adds the action.
        /// </summary>
        /// <param name="action">The action.</param>
        public void AddFirst(Action action)
        {
            _actions.AddFirst(action);
        }

        /// <summary>
        /// Adds the action.
        /// </summary>
        /// <param name="action">The action.</param>
        public void AddFirst(int node, bool stopAtNode, double waitTimeAfterStop)
        {
            _actions.AddFirst(new Action()
            {
                Node = node,
                StopAtNode = stopAtNode,
                WaitTimeAfterStop = waitTimeAfterStop
            });
        }

        /// <summary>
        /// Adds the action.
        /// </summary>
        /// <param name="action">The action.</param>
        public void AddLast(Action action)
        {
            _actions.AddLast(action);
        }

        /// <summary>
        /// Adds the action.
        /// </summary>
        /// <param name="action">The action.</param>
        public void AddLast(int node, bool stopAtNode, double waitTimeAfterStop)
        {
            _actions.AddLast(new Action()
            {
                Node = node,
                StopAtNode = stopAtNode,
                WaitTimeAfterStop = waitTimeAfterStop
            });
        }

        /// <summary>
        /// Removes the first action.
        /// </summary>
        public void RemoveFirstAction()
        {
            _actions.RemoveFirst();
        }


        /// <summary>
        /// Clears this instance.
        /// </summary>
        public void Clear()
        {
            _actions.Clear();
        }

        /// <summary>
        /// Action
        /// </summary>
        public class Action
        {

            /// <summary>
            /// next node
            /// </summary>
            public int Node;

            /// <summary>
            /// stop at the next node
            /// </summary>
            public bool StopAtNode;

            /// <summary>
            /// Time to wait at the destination. Should be 0, if StopAtDestination = false
            /// </summary>
            public double WaitTimeAfterStop;

            /// <summary>
            /// Returns a <see cref="System.String" /> that represents this instance.
            /// </summary>
            /// <returns>
            /// A <see cref="System.String" /> that represents this instance.
            /// </returns>
            public override string ToString()
            // public override means that the method is not implemented here but in a derived class
            {
                StringBuilder sb = new StringBuilder();
                sb.Append("Goto ").Append(Node); // example: Goto 1
                if (StopAtNode)
                    sb.Append(" and Stop"); // example: Goto 1 and Stop
                if (WaitTimeAfterStop > 0)
                    sb.Append(" and Wait ").Append(WaitTimeAfterStop); // example: Goto 1 and Stop and Wait 5
                return sb.ToString(); // example: Goto 1 and Stop and Wait 5
            }
        }

        /// <summary>
        /// Returns a <see cref="System.String" /> that represents this instance.
        /// </summary>
        /// <returns>
        /// A <see cref="System.String" /> that represents this instance.
        /// </returns>
        public override string ToString()
        // override means that the method is not implemented here but in a derived class
        {
            StringBuilder sb = new StringBuilder(); // stringbuilder is used to build a string
            foreach (var action in _actions)
            {
                sb.Append(action.ToString()).Append("; ");
            }
            return sb.ToString(); // 
        }

        /// <summary>
        /// Deletes the stop flag in position index if possible.
        /// translate abouve message to chinese: 如果可能，删除位置索引中的停止标志
        /// </summary>
        /// <param name="Graph">The graph.</param>
        /// <param name="index">The index.</param>
        /// <returns>stop flag deleted</returns>
        internal bool DeleteStopIfPossible(Graph Graph, int index)
        {
            //only the pre-last node is possible
            if (index != Count - 2)
                throw new NotSupportedException("No need to support it yet!");

            //get the nodes
            var lastAction = _actions.Last.Value; // last action is the last action in the list
            var preLastAction = _actions.Last.Previous.Value; // prelast action is the action before the last action in the list
            var prepreLastAction = _actions.Last.Previous.Previous.Value; // preprelast action is the action before the prelast action in the list

            //get the incoming and outgoing edge
            //translate abouve message to chinese: 获取传入和传出边缘
            var incomingEdge = Graph.BackwardEdges[preLastAction.Node].FirstOrDefault(e => e.From == prepreLastAction.Node); // backward edges are the edges that are going backwards
            var outgoingEdge = Graph.Edges[preLastAction.Node].FirstOrDefault(e => e.To == lastAction.Node); // edges are the edges that are going forward

            //edges must be found and the angle must equal
            if (incomingEdge == null || outgoingEdge == null || incomingEdge.Angle != outgoingEdge.Angle)
                return false;

            preLastAction.StopAtNode = false;
            return true;
        }

        /// <summary>
        /// Sets the stop on defined node.
        /// translate abouve message to chinese: 在定义的节点上设置停止
        /// </summary>
        /// <param name="collidesOnNode">The collides on node.</param>
        /// <exception cref="System.NotImplementedException"></exception>
        internal void SetStopBeforeNode(int node)
        {
            LinkedListNode<Action> currentNode = _actions.First; // current node is the first node in the list, example of current node: Goto 1 and Stop and Wait 5

            while (currentNode != null && currentNode.Next != null)
            {
                if (currentNode.Next.Value.Node == node)
                    currentNode.Value.StopAtNode = true;
                currentNode = currentNode.Next;
            }
        }
    }

    #endregion
}
