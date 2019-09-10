namespace L1PathFinder
{
    using System.Linq;

    //Vertices have to do multiple things
    //
    //  1.  They store the topology of the graph which is gonna get searched
    //  2.  They implement the pairing heap data sturcture (intrusively)
    //  3.  They implement a linked list for tracking clean up
    //  4.  Track search information (keep track of predecessors, distances, open state)
    //

    static class LandmarksHelper
    {
        public const int NUM_LANDMARKS = 16;

        public static double[] BuildEmptyLandMarks()
        {
            return Enumerable.Range(0, NUM_LANDMARKS).Select(i => double.MaxValue).ToArray();
        }
    }
}