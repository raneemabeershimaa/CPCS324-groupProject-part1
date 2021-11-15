/**
 * @author Abeer Hadadi , Raneem Saaty  ,Shaima Alharbi
 * @version 8.2
 * @since 15-11-2021
 */
package mst7;

import static java.lang.System.in;
import javafx.util.Pair;
import java.util.*;

public class MST7 {

    /**
     * Set the source, destination and weight
     */
    static class Edge implements Comparable<Edge> {

        /**
         * Stores source of a vertex in the adjacency list
         */
        int source;

        /**
         * Stores destination vertex in adjacency list
         */
        int destination;

        /**
         * Stores weight of a vertex in the adjacency list
         */
        int weight;

        /**
         * To Set values of the source, destination and weight
         *
         * @param source change source
         * @param destination coordinate destination
         * @param weight coordinate weight
         */
        public Edge(int source, int destination, int weight) {
            this.source = source;
            this.destination = destination;
            this.weight = weight;
        }

        /**
         * Compares the Number object that invoked the method to the argument
         *
         * @param t coordinate t
         * @return If the number is equal to the argument return 0, If the
         * number is less than the argument then return -1 or return 1 If the
         * number is greater than the argument
         */
        @Override
        public int compareTo(Edge t) {
            return this.weight - t.weight;
        }
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * HeapNode class
     */
    static class HeapNode {

        /**
         * vertex
         */
        int vertex;

        /**
         * The value in the heap which is always updated.
         */
        int key;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Implement Heaps by Illustrating Min Heap
     */
    static class MinHeap {

        /**
         * maximum possible size of min heap
         */
        int capacity;

        /**
         * Current number of elements in min heap
         */
        int size;

        /**
         * Member variables of this class
         */
        HeapNode[] heap;

        /**
         * used to decrease the key
         */
        int[] indexes; //will be used to decrease the key

        /**
         * Constructor of this class
         *
         * @param capacity coordinate capacity
         */
        public MinHeap(int capacity) {
            this.capacity = capacity;
            heap = new HeapNode[capacity + 1];
            indexes = new int[capacity];
            heap[0] = new HeapNode();
            heap[0].key = Integer.MIN_VALUE;
            heap[0].vertex = -1;
            size = 0;
        }

        /**
         * Check if it not empty
         *
         * @return true if it is not empty, otherwise false
         */
        public boolean isNotEmpty() {
            return size != 0;
        }

        /**
         * To add a node into the heap
         *
         * @param n coordinate heap[id]
         */
        public void add(HeapNode n) {
            size++;
            int id = size;
            heap[id] = n;
            indexes[n.vertex] = id;
            bubbleUp(id);
        }

        /**
         * To swap two nodes of the heap
         *
         * @param a coordinate HeapNode temp
         * @param b coordinate heap[a]
         */
        public void swap(int a, int b) {
            HeapNode temp = heap[a];
            heap[a] = heap[b];
            heap[b] = temp;
        }

        /**
         * moves the last value added to the correct position in the heap
         * heap. p/2 coordinate parent
         *
         * @param p coordinate current
         */
        public void bubbleUp(int p) {
            int parent = p / 2;
            int current = p;
            while (current > 0 && heap[parent].key > heap[current].key) {
                HeapNode currentNode = heap[current];
                HeapNode parentNode = heap[parent];

                //swap the positions
                indexes[currentNode.vertex] = parent;
                indexes[parentNode.vertex] = current;
                swap(current, parent);
                current = parent;
                parent = parent / 2;
            }
        }

        /**
         * To remove and return the minimum element from the heap
         *
         * @return
         */
        public HeapNode removeMin() {
            HeapNode min = heap[1];
            HeapNode last = heap[size];
            //update the indexes[] and move the last node to the top
            indexes[last.vertex] = 1;
            heap[1] = last;
            heap[size] = null;
            sinkDown(1);
            size--;
            return min;
        }

        /**
         * To ensure that the values in the min heap are in the correct order
         * rightChild coordinate 2 * s + 1 leftChild coordinate 2 * s
         *
         * @param s coordinate smallest
         */
        public void sinkDown(int s) {
            int rightChild = 2 * s + 1;
            int leftChild = 2 * s;
            int smallest = s;
            if (leftChild < heapSize() && heap[smallest].key > heap[leftChild].key) {
                smallest = leftChild;
            }
            if (rightChild < heapSize() && heap[smallest].key > heap[rightChild].key) {
                smallest = rightChild;
            }
            if (s != smallest) {
                HeapNode smallestNode = heap[smallest];
                HeapNode kthNode = heap[s];

                //swap the positions
                indexes[smallestNode.vertex] = s;
                indexes[kthNode.vertex] = smallest;
                swap(s, smallest);
                sinkDown(smallest);
            }
        }

        /**
         * To return the heap size
         *
         * @return size of heap
         */
        public int heapSize() {
            return size;
        }
    }

    /**
     * To create a graph
     */
    static class Graph {

        /**
         * List of adjacent nodes of a given vertex
         */
        LinkedList<Edge>[] vertsList; // List of adjacent nodes of a given vertex

        /**
         * Number of vertices in the graph
         */
        int NumOfVertices;

        /**
         * Number of edges in the graph
         */
        int NumOfEdges;

        /**
         * set as Undirected graph
         */
        boolean digraph = false;

        /**
         * This is a graph constructor to make graph with given number of
         * vertices and edges
         *
         * @param NumOfVertices number of vertices
         * @param NumOfEdges number of edges
         */
        Graph(int NumOfVertices, int NumOfEdges) {
            this.NumOfVertices = NumOfVertices;
            this.NumOfEdges = NumOfEdges;
            vertsList = new LinkedList[NumOfVertices];
            //initialize adjacency lists for all the vertices
            for (int i = 0; i < NumOfVertices; i++) {
                vertsList[i] = new LinkedList<>();
            }
        }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * This method create an object of type Edge from source to destination
         * to add it as first edge in adjacency list and create an edge from
         * destination to the source for undirected graph then add it between
         * two vertices
         *
         * @param source as an index in vertsList to connect it to destination
         * @param destination as an index in vertsList source
         * @param weight coordinate weight of an edge
         */
        public void addEdge(int source, int destination, int weight) {
            vertsList[source].add(new Edge(source, destination, weight)); // add it as first edge in adjacency list
            vertsList[destination].add(new Edge(destination, source, weight)); //for undirected graph
        }

        /**
         * mstSet class
         */
        static class mstSet {

            /**
             * parent of the vertex 
             */
            int parent;

            /**
             * weight of the edge 
             */
            int weight;
        }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * This method used to find the Minimum Spanning Tree by using
         * priority-queue based Prim algorithm
         *
         * @return cost of the MST
         */
        public int primPQ() {
            //initial cost of the MST
            int cost = 0;
            int numOfVertices = NumOfVertices;

            //Initialize array to check Whether a vertex is in PriorityQueue or not
            boolean[] minInMST = new boolean[numOfVertices];
            mstSet[] set = new mstSet[numOfVertices]; //create array to save for mstSet
            int[] keys = new int[numOfVertices];

            for (int i = 0; i < numOfVertices; i++) {
                keys[i] = Integer.MAX_VALUE; // Initialize key values to infinity
                set[i] = new mstSet();//initialize mstSet for all the vertices
            }

            PriorityQueue<Pair<Integer, Integer>> PriorityQueuePrim = new PriorityQueue<>(numOfVertices, new Comparator<Pair<Integer, Integer>>() {
                @Override
                // Comparator created for PriorityQueue
                // returns 1 if p1.key > p2.key
                // returns 0 if p1.key < p12.key and
                // returns -1 otherwise
                public int compare(Pair<Integer, Integer> p1, Pair<Integer, Integer> p2) {
                    return p1.getKey() - p2.getKey();
                }
            });

            //add the pair V0,(0.0) to PriorityQueue
            PriorityQueuePrim.add(new Pair<>(0, 0));

            set[0] = new mstSet();
            set[0].parent = -1;

            //keep until the PriorityQueue is empty
            while (PriorityQueuePrim.isEmpty() == false) {
                //get the element at the head of PriorityQueuePrim which is min pair
                int minPair = PriorityQueuePrim.poll().getValue();
                minInMST[minPair] = true; // Include that node into mstset

                LinkedList<Edge> linkList = vertsList[minPair];
                for (int i = 0; i < linkList.size(); i++) { //For all adjacent vertex of the extracted vertex 
                    //If V is in queue
                    if (minInMST[linkList.get(i).destination] == false) {
                        // If the key value of the adjacent vertex is
                        // more than the extracted key
                        if (keys[linkList.get(i).destination] > linkList.get(i).weight) {
                            //add it to the priority queue
                            PriorityQueuePrim.offer(new Pair<>(linkList.get(i).weight, linkList.get(i).destination));
                            // update the key value of adjacent vertex
                            // to update first remove and add the updated vertex
                            set[linkList.get(i).destination].parent = minPair;
                            set[linkList.get(i).destination].weight = linkList.get(i).weight;
                            keys[linkList.get(i).destination] = linkList.get(i).weight;
                        }
                    }
                }
            }
            //return Cost of primPQ algorithm
            cost = costMST(set);
            return cost;
        }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * To calculate the total MST
         *
         * @param mstset to reach the weight then coordinate cost
         * @return total MST
         */
        public int costMST(mstSet[] mstset) {
            int cost = 0;
            for (int i = 1; i < NumOfVertices; i++) {
                cost += mstset[i].weight;
            }
            return cost;
        }

/////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * The make_graph program implements an application that makes random
         * graphs with random edges assigned to random vertices. Edges have
         * random weights (0-20)
         *
         * @param graph with specific number of vertices and edges
         */
        public void make_graph(Graph graph) {
            int remaningEdge = NumOfEdges - (NumOfVertices - 1), vertex = 0, i = 0;
            Random randomNum = new Random();

            //All vertices are connected
            while (vertex < NumOfVertices - 1) {
                addEdge(vertex, vertex + 1, randomNum.nextInt(20) + 1);
                vertex++;
            }

            int NumOfVertices = graph.NumOfVertices;
            while (i < remaningEdge) {
                int source = randomNum.nextInt(NumOfVertices);
                int dest = randomNum.nextInt(NumOfVertices);
                if (dest == source || isConnected(source, dest, graph.vertsList)) { // to be sure no node have edge with himself 
                    i--;
                    continue;
                }
                addEdge(source, dest, randomNum.nextInt(20) + 1);
                i++;
            }
        }

        /**
         * This method which checks if the source and destination connected
         *
         * @param source1 source vertex
         * @param destination1 destination destination vertex.
         * @param edges all edges in LinkedList
         * @return true if source and destination connected , otherwise false
         */
        public boolean isConnected(int source1, int destination1, LinkedList<Edge>[] edges) {
            boolean check = false;
            for (LinkedList<Edge> e : edges) {
                for (Edge edge : e) {
                    if ((edge.source == source1 && edge.destination == destination1) || (edge.source == destination1 && edge.destination == source1)) {
                        check = true;
                    }
                }
            }
            return check;
        }
//////////////////////////////////////////////////////////////////////////////////////////////////

        // method used to find the kruskalMSTâ€™s Minimum Spanning Tree
        /**
         * The main function to construct MST using Kruskal's algorithm based
         * Prim algorithm
         *
         * @return total MST of kruskal
         */
        public int kruskalMST() {
            // start time
            int cost = 0, i = 0, index = 0, j;
            String treeV = ""; // this variable is used only in tracing
            LinkedList<Edge>[] allEdges = vertsList.clone(); // modified data type from ArrayList to LinkedList
            PriorityQueue<Edge> pq = new PriorityQueue<>(NumOfEdges, Comparator.comparingInt(o -> o.weight));

            //add all the edges to priority queue, //sort the edges on weights
            while (i < allEdges.length) {
                j = 0;
                while (j < allEdges[i].size()) {
                    pq.add(allEdges[i].get(j));
                    j++;
                }

                i++;
            }

            int[] parent = new int[NumOfVertices];

            //makeset
            makeSet(parent);

            LinkedList<Edge> mst = new LinkedList<>();

            do {
                Edge edge = pq.remove();

                int x_set = find(parent, edge.source);
                int y_set = find(parent, edge.destination);
                //check if adding this edge creates a cycle
                if (x_set == y_set) {
                    //ignore, will create cycle
                } else {
                    //add it to our final result
                    mst.add(edge);
                    treeV += edge.toString() + "\n";
                    index++;
                    union(parent, x_set, y_set);
                }

            } while (index < NumOfVertices - 1 && !pq.isEmpty());

            cost = KurCost(mst);
            return cost;
        }

        /**
         * To create a new element with a parent pointer to itself.
         *
         * @param parent [i] coordinate i
         */
        public void makeSet(int[] parent) {
            int i = 0;
            //Make set- creating a new element with a parent pointer to itself.
            while (i < NumOfVertices) {
                parent[i] = i;
                i++;
            }
        }

        /**
         *chain of parent pointers from x upwards through the tree until an element is reached whose parent is itself
         * @param parent
         * @param vertex
         * @return vertex
         */
        public int find(int[] parent, int vertex) {
            //chain of parent pointers from x upwards through the tree
            // until an element is reached whose parent is itself
            if (parent[vertex] != vertex) {
                return find(parent, parent[vertex]);
            };
            return vertex;
        }

        /**
         * A function that does union of two sets
         *
         * @param parent
         * @param x
         * @param y
         */
        public void union(int[] parent, int x, int y) {
            int x_set_parent = find(parent, x);
            int y_set_parent = find(parent, y);
            //make x as parent of y
            parent[y_set_parent] = x_set_parent;
        }

        /**
         * To calculate total cost of kruskal method
         *
         * @param edgeList to reach get(i) then coordinate edge
         * @return total cost of kruskal method
         */
        public int KurCost(LinkedList<Edge> edgeList) {
            int cost = 0, i = 0;
            while (i < edgeList.size()) {
                Edge edge = edgeList.get(i);

                cost += edge.weight;
                i++;
            }
            return cost;
        }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////Prim using min heap

        /**
         * Method used to find the Prim using min heap
         *
         * @return total cost of min heap
         */
        public int minHeapPrim() {
            int cost = 0;
            boolean[] insideHeap = new boolean[NumOfVertices];
            mstSet[] resultSet = new mstSet[NumOfVertices];
            //keys[] used to store the key to know whether min hea update is required
            int[] key = new int[NumOfVertices];
            //create heapNode for all the vertices
            HeapNode[] heapNodes = new HeapNode[NumOfVertices];
            for (int i = 0; i < NumOfVertices; i++) {
                heapNodes[i] = new HeapNode();
                heapNodes[i].vertex = i;
                heapNodes[i].key = Integer.MAX_VALUE;
                resultSet[i] = new mstSet();
                resultSet[i].parent = -1;
                insideHeap[i] = true;
                key[i] = Integer.MAX_VALUE;
            }
            //decrease the key for the first index
            heapNodes[0].key = 0;

            //add all the vertices to the MinHeap
            MinHeap minHeap = new MinHeap(NumOfVertices);
            //add all the vertices to priority queue
            for (int j = 0; j < NumOfVertices; j++) {
                minHeap.add(heapNodes[j]);
            }
            //while minHeap is not empty
            while (minHeap.isNotEmpty()) {
                //extract the min
                HeapNode removedNode = minHeap.removeMin();

                //extracted vertex
                int removedVertex = removedNode.vertex;
                insideHeap[removedVertex] = false;

                //iterate through all the adjacent vertices
                LinkedList<Edge> list = vertsList[removedVertex];
                for (int i = 0; i < list.size(); i++) {
                    Edge edge = list.get(i);
                    //only if edge destination is present in heap
                    if (insideHeap[edge.destination]) {
                        int destination = edge.destination;
                        int newKey = edge.weight;
                        //check if updated key < existing key, if yes, update if
                        if (key[destination] > newKey) {
                            decreaseKey(minHeap, newKey, destination);
                            //update the parent node for destination
                            resultSet[destination].parent = removedVertex;
                            resultSet[destination].weight = newKey;
                            key[destination] = newKey;
                        }
                    }
                }
            }
            int totalCost = totalCostMinHeap(resultSet);
            return totalCost;
        }

        /**
         * To calculate the total cost of min heap
         *
         * @param resultSet to reach weight then coordinate totalCost
         * @return total cost of min heap
         */
        public int totalCostMinHeap(mstSet[] resultSet) {
            int totalCost = 0;
            for (int i = 1; i < NumOfVertices; i++) {
                totalCost += resultSet[i].weight;
            }
            return totalCost;
        }

        /**
         * To Remove the min value from the heap
         *
         * @param minHeap to reach indexes then coordinate index
         * @param newKey node.key
         * @param vertex as an index in minHeap.indexes[vertex]
         */
        public void decreaseKey(MinHeap minHeap, int newKey, int vertex) {
            //get the index thats key needs a decrease
            int index = minHeap.indexes[vertex];
            //update the node's value
            HeapNode node = minHeap.heap[index];
            node.key = newKey;
            minHeap.bubbleUp(index);
        }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    }

    /**
     * This is the main method which makes use of make_graph method
     *
     * @param args
     */
    public static void main(String[] args) {
        int n = 0;
        int m = 0;
        double startTime, endTime, runTime = 0;
        Scanner in = new Scanner(System.in);

        System.out.println("\n-------CPCS 324: Algorithms and Data Structures (II) Group Project “ Part I-------\n");

        System.out.println(" Number Of Vertices      Number Of Edges");
        System.out.println(" -------------------------------------------"
                + "\n  1-  n=1,000 \t\t  m=10,000"
                + "\n  2-  n=1,000 \t\t  m=15,000"
                + "\n  3-  n=1,000 \t\t  m=25,000"
                + "\n  4-  n=5,000 \t\t  m=15,000"
                + "\n  5-  n=5,000 \t\t  m=25,000"
                + "\n  6- n=10,000 \t\t  m=15,000"
                + "\n  7  n=10,000 \t\t  m=25,000"
                + "\n  8- n=20,000 \t\t  m=200,000"
                + "\n  9- n=20,000 \t\t  m=300,000"
                + "\n 10- n=50,000 \t\t  m=1,000,000");

        System.out.print("\n Select a case to start: ");
        int c = in.nextInt();
        switch (c) {
            case 1: {
                n = 1000;
                m = 10000;
            }
            break;
            case 2: {
                n = 1000;
                m = 15000;
            }
            break;
            case 3: {
                n = 1000;
                m = 25000;
            }
            break;
            case 4: {
                n = 5000;
                m = 15000;
            }
            break;
            case 5: {
                n = 5000;
                m = 25000;
            }
            break;
            case 6: {
                n = 10000;
                m = 15000;
            }
            break;
            case 7: {
                n = 10000;
                m = 25000;
            }
            break;
            case 8: {
                n = 20000;
                m = 200000;
            }
            break;
            case 9: {
                n = 20000;
                m = 300000;
            }
            break;
            case 10: {
                n = 50000;
                m = 1000000;
            }
            break;
            default:
                System.out.println("Wrong input !!");
        }

        Graph graph = new Graph(n, m);
        graph.make_graph(graph);

        System.out.println("\n------------------*Prim's Algorithm*--------------\n");
        startTime = System.currentTimeMillis();
        graph.primPQ();
        endTime = System.currentTimeMillis();
        runTime = endTime - startTime;
        System.out.println("Total runtime of Prim's Algorithm is: " + runTime + " ms");
        System.out.println("Total MST is: " + graph.primPQ() + "\n");
//-----------------------------------------------------------------------------------------------------
        System.out.println("------------------*Kruskal's Algorithm*--------------\n");

        startTime = System.currentTimeMillis();
        graph.kruskalMST();
        endTime = System.currentTimeMillis();
        runTime = endTime - startTime;
        System.out.println("Total runtime of Kruskal's Algorithm is: " + runTime + " ms");
        System.out.println("Total MST is: " + graph.kruskalMST() + "\n");
//----------------------------------------------------------------------------------------------------------        
        System.out.println("------------------*Prim's algorithm using min-heap*--------------\n");
        startTime = System.currentTimeMillis();
        graph.minHeapPrim();
        endTime = System.currentTimeMillis();
        runTime = endTime - startTime;
        System.out.println("Total runtime of Prim's algorithm using min-heap is: " + runTime + " ms");
        System.out.println("Total MST is: " + graph.minHeapPrim() + "\n");
    }
}
