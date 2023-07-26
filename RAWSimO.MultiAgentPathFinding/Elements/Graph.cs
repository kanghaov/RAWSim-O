using RAWSimO.Toolbox;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RAWSimO.MultiAgentPathFinding.Elements
{
    /// <summary>
    /// 2D space graph
    /// </summary>
    public class Graph
    {
        /// <summary>
        /// Number of nodes.
        /// translate above to chinese: 节点数量, such as 10
        /// </summary>
        public int NodeCount;

        /// <summary>
        /// x positions of nodes.
        /// translate above to chinese: 节点x坐标, such as 1,2,3,4,5,6,7,8,9,10
        /// </summary>
        public double[] PositionX;

        /// <summary>
        /// y positions of nodes.
        /// translate above to chinese: 节点y坐标, such as 1,2,3,4,5,6,7,8,9,10
        /// </summary>
        public double[] PositionY;

        /// <summary>
        /// Contains meta information objects for all nodes.
        /// translate above to chinese: 节点元信息,such as 
        /// </summary>
        public NodeInfo[] NodeInfo;

        /// <summary>
        /// Edges of the graph.
        /// translate above to chinese: 节点边
        /// [nodeID][edgeID]
        /// </summary>
        public Edge[][] Edges;

        /// <summary>
        /// Edges of the graph.
        /// translate above to chinese: 节点边
        /// [nodeID][edgeID]
        /// </summary>
        public Edge[][] BackwardEdges;

        /// <summary>
        /// Edges of the graph.
        /// translate above to chinese: 节点边
        /// [nodeID][edgeID]
        /// </summary>
        public Dictionary<int, ElevatorEdge[]> ElevatorEdges;

        /// <summary>
        /// Create a graph.
        /// translate above to chinese: 创建一个图
        /// </summary>
        /// <param name="nodeCount">sum of nodes</param>
        public Graph(int nodeCount)
        {

            // NodeInfo = new NodeInfo[nodeCount];：這裡，new 用於創建一個NodeInfo對象的數組。數組的長度由變量nodeCount決定。

            // Edges = new Edge[nodeCount][];：在此，new 用於創建一個二維數組Edges。該數組的第一維的大小為nodeCount，第二維的大小還未被指定。每個元素都是Edge類型的數組。

            // PositionX = new double[nodeCount]; 和 PositionY = new double[nodeCount];：在這兩行中，new 創建了兩個double類型的數組，名為PositionX 和 PositionY，每個數組的長度都由變量nodeCount決定。

            // ElevatorEdges = new Dictionary<int, ElevatorEdge[]>();：在這裡，new 關鍵字用於實例化一個新的Dictionary<int, ElevatorEdge[]>對象，名為ElevatorEdges。

            NodeCount = nodeCount; // example of nodeCount: 10
            NodeInfo = new NodeInfo[nodeCount]; // example of NodeInfo's structure: NodeInfo[0] = {ID = 0, IsLocked = false, IsObstacle = false, IsQueue = false, QueueTerminal = -1}
            Edges = new Edge[nodeCount][]; // example of Edges's structure: Edges[0] = {Edge[0], Edge[1], Edge[2], Edge[3], Edge[4], Edge[5], Edge[6], Edge[7], Edge[8], Edge[9]}
            PositionX = new double[nodeCount]; // example of PositionX's structure: PositionX[0] = 1, PositionX[1] = 2, PositionX[2] = 3, PositionX[3] = 4, PositionX[4] = 5, PositionX[5] = 6, PositionX[6] = 7, PositionX[7] = 8, PositionX[8] = 9, PositionX[9] = 10
            PositionY = new double[nodeCount]; // example of PositionY's structure: PositionY[0] = 1, PositionY[1] = 2, PositionY[2] = 3, PositionY[3] = 4, PositionY[4] = 5, PositionY[5] = 6, PositionY[6] = 7, PositionY[7] = 8, PositionY[8] = 9, PositionY[9] = 10
            ElevatorEdges = new Dictionary<int, ElevatorEdge[]>(); // example of ElevatorEdges's structure: ElevatorEdges[0] = {ElevatorEdge[0], ElevatorEdge[1], ElevatorEdge[2], ElevatorEdge[3], ElevatorEdge[4], ElevatorEdge[5], ElevatorEdge[6], ElevatorEdge[7], ElevatorEdge[8], ElevatorEdge[9]}
        }

        /// <summary>
        /// Generate all backward edges.
        /// translate above to chinese: 生成所有的反向边
        /// </summary>
        public void GenerateBackwardEgdes()
        {

            // 在 C# 程序设计语言中，`new`是一个关键字，用于在堆内存中实例化对象或创建数组。

            // 在你提供的代码中，`new`的用法有两个场景：

            // 1. `BackwardEdges = new Edge[NodeCount][];`：在这一行中，`new`关键字用于创建一个二维数组`BackwardEdges`。数组的第一维大小为`NodeCount`，第二维的大小还未指定。每个元素都是`Edge`类型的数组。

            // 2. `Dictionary<int, List<Edge>> dynBackwardEdges = new Dictionary<int, List<Edge>>();` 和 `dynBackwardEdges[nodeId] = new List<Edge>();`：在这两行中，`new`关键字用于实例化对象。第一行创建了一个新的`Dictionary<int, List<Edge>>`对象，称为`dynBackwardEdges`。第二行给字典`dynBackwardEdges`的每个`nodeId`键分配了一个新的`List<Edge>`对象。

            // 总的来说，`new`关键字在 C# 中用于创建新的对象实例或数组。它会在堆内存中分配足够的空间来存储新对象或数组，并返回一个指向它的引用。对于对象，`new`关键字也会调用对象的构造函数，进行初始化。

            //create array
            // example of BackwardEdges's structure: BackwardEdges[0] = {Edge[0], Edge[1], Edge[2], Edge[3], Edge[4], Edge[5], Edge[6], Edge[7], Edge[8], Edge[9]}
            BackwardEdges = new Edge[NodeCount][];

            //use a temporary data structure with dynamic memory allocation
            Dictionary<int, List<Edge>> dynBackwardEdges = new Dictionary<int, List<Edge>>();
            for (int nodeId = 0; nodeId < NodeCount; nodeId++)
                dynBackwardEdges[nodeId] = new List<Edge>();

            //fill dynamic Data Structure
            for (int nodeId = 0; nodeId < NodeCount; nodeId++)
                for (int edgeId = 0; edgeId < Edges[nodeId].Length; edgeId++)
                    dynBackwardEdges[Edges[nodeId][edgeId].To].Add(Edges[nodeId][edgeId]);

            //create Backward Edges
            for (int nodeId = 0; nodeId < NodeCount; nodeId++)
                BackwardEdges[nodeId] = dynBackwardEdges[nodeId].ToArray();

            //this helps the GC
            dynBackwardEdges.Clear();

        }

        /// <summary>
        /// Gets the intermediate nodes.
        /// translate above to chinese: 获取中间节点
        /// </summary>
        /// <param name="node1">The node1.</param>
        /// <param name="node2">The node2.</param>
        /// <returns>intermediate nodes</returns>
        public List<int> getIntermediateNodes(int node1, int node2)
        {
            var intermediateNodes = new List<int>(); // example of intermediateNodes's structure: intermediateNodes[0] = 1, intermediateNodes[1] = 2, intermediateNodes[2] = 3, intermediateNodes[3] = 4, intermediateNodes[4] = 5, intermediateNodes[5] = 6, intermediateNodes[6] = 7, intermediateNodes[7] = 8, intermediateNodes[8] = 9
            var angle = Graph.RadToDegree(Math.Atan2(PositionY[node2] - PositionY[node1], PositionX[node2] - PositionX[node1])); // example of angle's structure: angle = 45

            var node = node1;

            //loop from node1 to node2
            while (node != node2)
            {
                try
                {
                    node = Edges[node].Where(e => Math.Abs(e.Angle - angle) < 10).First().To;
                    // 代码首先使用节点变量作为键来访问 Edges 字典。Edges 字典的值是与当前节点相连的边的列表。

                    // 然后，在边的列表上调用 Where 方法，以过滤不匹配给定角度的边。Where 方法接受一个 lambda 表达式作为参数，该表达式将每个边的角度与给定角度进行比较。使用 Math.Abs 方法获取两个角度之间的绝对差值。如果绝对差值小于 10，则认为该边是匹配的。

                    // 接下来，在过滤后的边列表上调用 First 方法，以获取第一个匹配给定角度的边。然后返回该边的 To 属性，该属性表示图中的下一个节点。

                    // 总的来说，这行代码用于根据给定的角度找到图中的下一个节点。它通过过滤不匹配给定角度的边，并根据第一个匹配的边返回图中的下一个节点来实现这一点。
                }
                catch (Exception)
                {
                    return null;
                }

                if (node != node2)
                    intermediateNodes.Add(node);
            }



            return intermediateNodes;
        }

        /// <summary>
        /// Gets the node distance.
        /// translate above to chinese: 获取节点距离
        /// </summary>
        /// <param name="node1">The node1.</param>
        /// <param name="node2">The node2.</param>
        /// <returns></returns>
        public double getDistance(int node1, int node2)
        {
            return Math.Sqrt((PositionX[node1] - PositionX[node2]) * (PositionX[node1] - PositionX[node2]) + (PositionY[node1] - PositionY[node2]) * (PositionY[node1] - PositionY[node2]));
        }

        /// <summary>
        /// Gets the squared node distance.
        /// translate above to chinese: 获取节点距离的平方
        /// </summary>
        /// <param name="node1">The node1.</param>
        /// <param name="node2">The node2.</param>
        /// <returns></returns>
        public double getSquaredDistance(int node1, int node2)
        {
            return (PositionX[node1] - PositionX[node2]) * (PositionX[node1] - PositionX[node2]) + (PositionY[node1] - PositionY[node2]) * (PositionY[node1] - PositionY[node2]);
        }

        /// <summary>
        /// Contains 2 times pi as a constant.
        /// </summary>
        public const double PI2 = Math.PI * 2;

        /// <summary>
        /// Convert degrees to rad.
        /// </summary>
        /// <param name="degree">degree</param>
        /// <returns>rad</returns>
        public static double DegreeToRad(int degree)
        {
            var rad = degree * PI2 / 360.0;
            rad = (rad + (PI2)) % (PI2); // make sure rad is in [0, 2pi]
            return rad;
        }

        /// <summary>
        /// Convert rad to degrees.
        /// </summary>
        /// <param name="rad">rad</param>
        /// <returns>degrees</returns>
        public static short RadToDegree(double rad)
        {
            var degree = (int)Math.Round(rad * 360 / (PI2));
            return (short)((degree + 360) % 360);
        }
    }
}
