using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RAWSimO.MultiAgentPathFinding.Elements
{

    /// <summary>
    /// An edge of a graph
    /// translate above to chinese: 图的边
    /// </summary>
    public class Edge
    {
        /// <summary>
        /// from node
        /// translate above to chinese: 起始节点
        /// </summary>
        public int From;

        /// <summary>
        /// to node
        /// translate above to chinese: 终止节点
        /// </summary>
        public int To;

        /// <summary>
        /// distance from fromNode to toNode in meter
        /// translate above to chinese: 起始节点到终止节点的距离
        /// </summary>
        public double Distance;

        /// <summary>
        /// Angle of the edge 0°-360°
        /// 0° = East and is increasing clockwise
        /// translate above to chinese: 边的角度 0°-360° 0° = 东方，顺时针增加
        /// </summary>
        public short Angle;

        /// <summary>
        /// Contains meta information about the from part of the edge.
        /// translate above to chinese: 包含有关边缘起始部分的元信息。
        /// </summary>
        public NodeInfo FromNodeInfo;
        /// <summary>
        /// Contains meta information about the to part of the edge.
        /// translate above to chinese: 包含有关边缘终止部分的元信息。
        /// </summary>
        public NodeInfo ToNodeInfo;
    }

    /// <summary>
    /// An elevator edge of a graph
    /// translate above to chinese: 图的电梯边
    /// </summary>
    public class ElevatorEdge : Edge
    {
        /// <summary>
        /// distance from fromNode to toNode in meter
        /// translate above to chinese: 起始节点到终止节点的距离
        /// </summary>
        public double TimeTravel;

        /// <summary>
        /// The reference to the elevator
        /// translate above to chinese: 电梯的引用
        /// </summary>
        public object Reference;
    }

}
