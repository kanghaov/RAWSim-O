using RAWSimO.MultiAgentPathFinding.Algorithms.AStar;
using RAWSimO.MultiAgentPathFinding.DataStructures;
using RAWSimO.MultiAgentPathFinding.Elements;
using RAWSimO.MultiAgentPathFinding.Physic;
using RAWSimO.MultiAgentPathFinding.Toolbox;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RAWSimO.MultiAgentPathFinding.Methods
{

    /// <summary>
    /// Conflict-based for optimal multi-agent pathﬁnding, Sharon 2015
    /// translates above into chinese: 基于冲突的最优多智能体路径规划
    /// </summary>
    public class CBSMethod : PathFinder
    // 在C#中，public class CBSMethod : PathFinder表示定义了一个名为CBSMethod的公开类，该类继承自PathFinder类。下面是对这段代码的详细解释：

    // public：这是一个访问修饰符，它决定了类的可见性。public表示该类可以被任何其他类访问。

    // class：这是一个关键字，用于声明一个类。

    // CBSMethod：这是类的名称。在这个例子中，类的名称是CBSMethod。

    // :：这个符号在这个上下文中表示继承。在C#中，单冒号用于表示单一继承，即一个类只能继承一个基类。

    // PathFinder：这是基类的名称。在这个例子中，CBSMethod类是从PathFinder类继承的。

    // 继承是面向对象编程的一个重要特性，它允许创建一个新类（子类或派生类）来使用已存在的类（父类或基类）的成员（例如字段和方法）。继承的主要目的是为了代码复用和组织代码。它也可以用来表示“is-a”类型的关系，例如在这个例子中，CBSMethod是一种PathFinder。
    {
        /// <summary>
        /// The search method for node selection
        /// </summary>
        public CBSSearchMethod SearchMethod = CBSSearchMethod.BestFirst;

        /// <summary>
        /// The reservation table for finding a way through constraints
        /// translation: 用于通过约束找到一条路径的预约表
        /// </summary>
        private ReservationTable _reservationTable;

        /// <summary>
        /// The reservation table for collision detection
        /// translation: 用于碰撞检测的预约表
        /// </summary>
        ReservationTable _agentReservationTable;

        /// 总的来说，下划线 _ 不是关键字或访问修饰符，它只是一种命名约定，用于表示私有字段或私有属性。在 C# 中，通常使用下划线 _ 作为私有字段的前缀，以区分它们与公共字段或属性。

        /// <summary>
        /// Lambda Express for node selection
        /// translation: 用于节点选择的Lambda表达式
        /// 声明了一个名为 NodeSelectionExpression 的委托。委托是一种表示方法签名的类型，可用于将方法作为参数传递给其他方法或将方法存储为变量。
        /// NodeSelectionExpression 委托接受一个 ConflictTree.Node 类型的参数，并返回一个 double 值。该委托的目的是定义一个方法，该方法可以根据某些标准选择冲突树中的节点
        /// </summary>
        /// <param name="node">The node.</param>
        /// <returns></returns>
        delegate double NodeSelectionExpression(ConflictTree.Node node);

        /// <summary>
        /// The deadlock handler
        /// translation: 死锁处理程序
        /// </summary>
        private DeadlockHandler _deadlockHandler;

        /// <summary>
        /// Initializes a new instance of the <see cref="FARMethod"/> class.
        /// </summary>
        /// <param name="graph">graph</param>
        /// <param name="seed">The seed to use for the randomizer.</param>
        /// <param name="logger">The logger to use.</param>
        public CBSMethod(Graph graph, int seed, PathPlanningCommunicator logger)
            : base(graph, seed, logger) // what is base:https://blog.csdn.net/yiyelanxin/article/details/88570151
        {
            if (graph.BackwardEdges == null)
                graph.GenerateBackwardEgdes();
            _reservationTable = new ReservationTable(graph); // _reservationTable是一个ReservationTable类型的变量，它是一个预约表，用于找到一条路径通过约束
            _agentReservationTable = new ReservationTable(graph, false, true, false); // _agentReservationTable是一个ReservationTable类型的变量，它是一个预约表，用于碰撞检测
            _deadlockHandler = new DeadlockHandler(graph, seed);
        }

        /// <summary>
        /// Find the path for all the agents.
        /// translation: 为所有智能体找到路径
        /// </summary>
        /// <param name="currentTime">The current Time.</param>
        /// <param name="agents">agents</param>
        public override void FindPaths(double currentTime, List<Agent> agents)
        {
            Stopwatch.Restart();

            //initialization data structures
            var conflictTree = new ConflictTree();
            var Open = new FibonacciHeap<double, ConflictTree.Node>();
            // GitHub Copilot: 您选择的代码是 CBSMethod.cs 文件中的一段代码，包括第 94 行。这段代码创建了一个新的 FibonacciHeap<double, ConflictTree.Node> 对象，并将其赋值给 Open 变量。

            // 在这段代码中，`double` 是一个数据类型，表示具有双精度的浮点数。它被用作 `FibonacciHeap` 类的 `TKey` 类型参数。`TKey` 参数指定堆中每个节点关联的优先级值的类型。

            // `ConflictTree.Node` 是 `ConflictTree` 类中的一个嵌套类。它表示冲突树中的一个节点，并包含与该节点相关的约束信息。它被用作 `FibonacciHeap` 类的 `TValue` 类型参数。`TValue` 参数指定堆中每个节点关联的值的类型。

            // 总的来说，这段代码创建了一个新的 FibonacciHeap<double, ConflictTree.Node> 对象，并将其赋值给 Open 变量。`double` 用于表示堆中每个节点关联的优先级值，而 `ConflictTree.Node` 用于表示堆中每个节点关联的值。
            var solvable = true;
            var generatedNodes = 0;
            ConflictTree.Node bestNode = null;
            double bestTime = 0.0;

            //deadlock handling
            _deadlockHandler.LengthOfAWaitStep = LengthOfAWaitStep; // LengthOfAWaitStep = 5
            _deadlockHandler.MaximumWaitTime = 30; // MaximumWaitTime = 30
            _deadlockHandler.Update(agents, currentTime);

            //simply blocked
            foreach (var agent in agents.Where(a => a.FixedPosition)) // judge agent is stop by FixedPosition
                Graph.NodeInfo[agent.NextNode].IsLocked = true; // mark these agents is lock 

            // TODO this only works as long as a possible solution is guaranteed - maybe instead ignore paths to plan for agents with no possible solution and hope that it clears by others moving on?
            //first node initialization
            List<Agent> unsolvableAgents = null;
            foreach (var agent in agents.Where(a => !a.FixedPosition)) // agent is still moving
            {
                bool agentSolved = Solve(conflictTree.Root, currentTime, agent); // solve the agent
                if (!agentSolved) // agent is not solved
                {
                    // 使用了条件语句来检查 unsolvableAgents 变量是否为 null。如果 unsolvableAgents 变量为 null，则创建一个新的 List<Agent> 对象，并将当前智能体添加到该列表中。如果 unsolvableAgents 变量不为 null，则将当前智能体添加到现有的 List<Agent> 对象中。
                    if (unsolvableAgents == null) // unsolvableAgents is null
                        unsolvableAgents = new List<Agent>() { agent }; // structure of unsolvableAgents is a list,add current agent to unsolvableAgents
                    else
                        unsolvableAgents.Add(agent);
                }
                solvable = solvable && agentSolved; // solvable is true if all agents are solved
            }

            //node selection strategy (Queue will pick the node with minimum value
            //使用 NodeSelectionExpression 委托来选择下一个要扩展的节点。nodeObjectiveSelector 委托根据搜索方法的不同返回不同的值。如果搜索方法为 CBSSearchMethod.BestFirst，则返回节点的解决方案成本。如果搜索方法为 CBSSearchMethod.BreathFirst，则返回节点的深度。如果搜索方法为 CBSSearchMethod.DepthFirst，则返回节点的负深度。

            //NodeSelectionExpression 委托是一个函数，它接受一个 Node 对象作为输入，并返回一个数字值。该数字值表示用于选择 CBS 算法中要扩展的下一个节点的目标函数。
            //nodeObjectiveSelector 委托的类型是 NodeSelectionExpression，它是一个函数类型，接受一个 Node 对象作为输入，并返回一个数字值。在这里，nodeObjectiveSelector 委托的实现是一个 lambda 表达式，它接受一个 Node 对象作为输入，并将其直接返回。这意味着在这种情况下，nodeObjectiveSelector 委托不会改变选择下一个节点的目标函数。

            //在 C# 中，= 是赋值运算符，用于将右侧的值赋给左侧的变量。例如，int x = 5; 将整数值 5 赋给变量 x。
            // 而 => 是 lambda 表达式的语法。lambda 表达式是一种匿名函数，它可以作为参数传递给其他函数或委托。lambda 表达式的语法是 input => expression，其中 input 是输入参数，expression 是函数体。例如，(x, y) => x + y 是一个 lambda 表达式，它接受两个整数参数 x 和 y，并返回它们的和。
            // 在这段代码中，= 是用于将 lambda 表达式赋值给 nodeObjectiveSelector 变量的赋值运算符。而 => 是 lambda 表达式的语法，用于定义 lambda 表达式的输入参数和函数体。
            NodeSelectionExpression nodeObjectiveSelector = node =>
            {
                switch (SearchMethod)
                {
                    case CBSSearchMethod.BestFirst:
                        return node.SolutionCost;
                    case CBSSearchMethod.BreathFirst:
                        return node.Depth;
                    case CBSSearchMethod.DepthFirst:
                        return (-1) * node.Depth;
                    default:
                        return 0;
                }
            };

            //Enqueue first node, enqueue in chinese is 入队
            // 使用了条件语句来检查 unsolvableAgents 变量是否为 null。如果 unsolvableAgents 变量为 null，则将冲突树的根节点添加到 Open 变量中。如果 unsolvableAgents 变量不为 null，则将冲突树的根节点添加到 Open 变量中，并将其解决方案成本设置为正无穷大。
            if (solvable) // solvable is true if all agents are solved
                Open.Enqueue(conflictTree.Root.SolutionCost, conflictTree.Root); // enqueue the root node
            else
                Communicator.LogDefault("WARNING! Aborting CBS - could not obtain an initial solution for the following agents: " +
                    string.Join(",", unsolvableAgents.Select(a => "Agent" + a.ID.ToString() + "(" + a.NextNode.ToString() + "->" + a.DestinationNode.ToString() + ")")));
            bestNode = conflictTree.Root; // why the conflictTree.Root is the bestNode? because the conflictTree.Root is the first node,

            //search loop
            ConflictTree.Node p = conflictTree.Root; // p is the current node
            while (Open.Count > 0) // > 0 means that the conflict agent is not solved
            {

                //local variables
                int agentId1;
                int agentId2; // agentId1 and agentId2 are the agents that conflict
                ReservationTable.Interval interval;

                //pop out best node
                p = Open.Dequeue().Value; // p is agent? or node? p is node,like the root node

                //check the path
                // hasNoConflicts is a bool value, if the path has no conflicts, then hasNoConflicts is true
                var hasNoConflicts = ValidatePath(p, agents, out agentId1, out agentId2, out interval); // p is a path list , agents is a list of agents, out agentId1 and agentId2 are the agents that conflict, out interval is the interval that the agents conflict

                //has no conflicts?
                if (hasNoConflicts)
                {
                    bestNode = p;
                    break;
                }

                // time up? => return the best solution
                if (Stopwatch.ElapsedMilliseconds / 1000.0 > RuntimeLimitPerAgent * agents.Count * 0.9 || Stopwatch.ElapsedMilliseconds / 1000.0 > RunTimeLimitOverall)
                {
                    Communicator.SignalTimeout();
                    break;
                }

                //save best node
                if (bestNode == null || interval.Start > bestTime)
                {
                    bestTime = interval.Start;
                    bestNode = p;
                }

                //append child 1
                var node1 = new ConflictTree.Node(agentId1, interval, p);
                solvable = Solve(node1, currentTime, agents.First(a => a.ID == agentId1));
                if (solvable)
                    Open.Enqueue(node1.SolutionCost, node1);

                //append child 2
                var node2 = new ConflictTree.Node(agentId2, interval, p);
                solvable = Solve(node2, currentTime, agents.First(a => a.ID == agentId2));
                if (solvable)
                    Open.Enqueue(node2.SolutionCost, node2);

                generatedNodes += 2;

            }

            //return the solution => suboptimal
            foreach (var agent in agents)
            {
                agent.Path = bestNode.getSolution(agent.ID);
                if (_deadlockHandler.IsInDeadlock(agent, currentTime))
                    _deadlockHandler.RandomHop(agent);
            }
        }

        /// this function is used to find out the agrnt that can solve the conflict
        private bool ValidatePath(ConflictTree.Node node, List<Agent> agents, out int agentId1, out int agentId2, out ReservationTable.Interval interval)
        {
            //clear
            _agentReservationTable.Clear();

            //add next hop reservations
            foreach (var agent in agents.Where(a => !a.FixedPosition)) // agent is still moving
                _agentReservationTable.Add(agent.ReservationsToNextNode, agent.ID); // add the agent's reservation to the next node

            //get all reservations sorted
            var reservations = new FibonacciHeap<double, Tuple<Agent, ReservationTable.Interval>>(); // reservations is a FibonacciHeap<double, Tuple<Agent, ReservationTable.Interval>> type variable, it is a heap, and the key is double, the value is Tuple<Agent, ReservationTable.Interval>
            foreach (var agent in agents.Where(a => !a.FixedPosition)) // agent is still moving
                foreach (var reservation in node.getReservation(agent.ID))
                    reservations.Enqueue(reservation.Start, Tuple.Create(agent, reservation)); // for example, agent is agent1, reservation is (1,2), then the key is 1, the value is (agent1, (1,2))

            //check all reservations
            while (reservations.Count > 0)
            {
                var reservation = reservations.Dequeue().Value; // reservation is a tuple, the first element is agent, the second element is reservation

                int collideWithAgentId; // collideWithAgentId is the agent that conflict with the agent
                var intersectionFree = _agentReservationTable.IntersectionFree(reservation.Item2, out collideWithAgentId); // check if the reservation is intersection free, if it is not intersection free, then the collideWithAgentId is the agent that conflict with the agent
                if (!intersectionFree) // means that agent1 and agent2 conflict
                {
                    agentId1 = collideWithAgentId; // agentId1 is the agent that conflict with the agent
                    agentId2 = reservation.Item1.ID; // agentId2 is the original agent
                    interval = _agentReservationTable.GetOverlappingInterval(reservation.Item2);// interval is the interval that the agents conflict
                    if (interval.End - interval.Start > ReservationTable.TOLERANCE)
                        return false;
                }
                else
                {
                    _agentReservationTable.Add(reservation.Item2, reservation.Item1.ID);
                }
            }

            agentId1 = -1;
            agentId2 = -1;
            interval = null;
            return true;
        }


        /// <summary>
        /// Solves the specified node.
        /// translation: 解决了指定的节点
        /// </summary>
        /// <param name="node">The node.</param>
        /// <param name="currentTime">The current time.</param>
        /// <param name="agent">The agent.</param>
        /// <param name="obstacleNodes">The obstacle nodes.</param>
        /// <param name="lockedNodes">The locked nodes.</param>
        /// <returns></returns>
        private bool Solve(ConflictTree.Node node, double currentTime, Agent agent)
        {
            //clear reservation table
            _reservationTable.Clear();

            //add constraints (agentId = -1 => Intervals from the tree)
            foreach (var constraint in node.getConstraints(agent.ID))
                _reservationTable.Add(constraint.IntervalConstraint);

            //drive to next node must be possible - otherwise it is not a valid node
            foreach (var reservation in agent.ReservationsToNextNode)
                if (!_reservationTable.IntersectionFree(reservation))
                    return false;

            //We can use WHCA Star here in a low level approach.
            //Window = Infinitively long
            var rraStar = new ReverseResumableAStar(Graph, agent, agent.Physics, agent.DestinationNode);
            var aStar = new SpaceTimeAStar(Graph, LengthOfAWaitStep, double.PositiveInfinity, _reservationTable, agent, rraStar);

            //execute
            var found = aStar.Search();

            //+ WHCA* Nodes
            List<ReservationTable.Interval> reservations;
            Path path = new Path();
            if (found)
            {
                //add all WHCA Nodes
                aStar.GetPathAndReservations(ref path, out reservations);

#if DEBUG
                foreach (var reservation in reservations)
                    Debug.Assert(_reservationTable.IntersectionFree(reservation));
#endif

                //add the next node again
                if (path.Count == 0 || path.NextAction.Node != agent.NextNode || path.NextAction.StopAtNode == false)
                    path.AddFirst(agent.NextNode, true, 0);

                node.setSolution(agent.ID, path, reservations);

                //found
                return true;

            }
            else
            {
                //not found
                return false;
            }


        }

        /// <summary>
        /// Strategy for Node Selection
        /// </summary>
        [Serializable]
        public enum CBSSearchMethod
        {
            BestFirst,
            DepthFirst,
            BreathFirst
        }

    }
}
