import java.lang.reflect.Array;
import java.util.*;


public class Gwork{

//==================< COMMON >==============\\

    private static boolean[] marked;
    private static int[] edgeTo;
    private static int source;


//================( DFS )=============================\\

    public static void DFS(Graph G, Node S){
       marked = new boolean[G.V()];
       edgeTo = new int[G.V()];
       source = S.ID();
       dfs(G, source);
    }

    public static void DFS(Graph G, int S){
        Node node = new Node(S);
        DFS(G, node);
    }

    private static void dfs(Graph G, int s){
        Stack<Integer> stack = new Stack<>();

        stack.push(s);
        while(!stack.empty()){
            int v = stack.pop();
            if(!marked[v]){
                marked[v] = true;
                for(Edge e: G.adj(v)){
                    if(!marked[e.w()]){
                        edgeTo[e.w()] = e.v();
                        stack.push(e.w());
                    }
                }
            }
        }
    }

    public static boolean hasPathTo(int v){
        return marked[v];
    }

    public static Iterable<Integer> pathTo(int v){
        if(!hasPathTo(v)) return null;
        Stack<Integer> path = new Stack<>();
        for(int x=v; x!=source; x=edgeTo[x])
            path.push(x);
        path.push(source);
        ArrayList<Integer> list = new ArrayList<>(path);
        Collections.reverse(list);
        return list;
    }

//================( BFS )=============================\\

    public static void BFS(Graph G, Node S){
        marked = new boolean[G.V()];
        edgeTo = new int[G.V()];
        source = S.ID();
        bfs(G, source);
    }

    public static void BFS(Graph G, int S){
        Node node = new Node(S);
        BFS(G, node);
    }

    private static void bfs(Graph G, int s){
        Queue<Integer> queue = new LinkedList<>();

        marked[s] = true;
        queue.add(s);
        while(!queue.isEmpty()){
            int v = queue.remove();
            for(Edge e: G.adj(v)){
                if(!marked[e.w()]){
                    edgeTo[e.w()] = e.v();
                    marked[e.w()] = true;
                    queue.add(e.w());
                }
            }
        }
    }

    public static boolean hasPathTo(Graph g, int u, int v){
        BFS(g, u);
        return hasPathTo(v);
    }

    public static boolean hasPathTo(Graph g, Node A, Node B){
        BFS(g, A.ID());
        return hasPathTo(B.ID());
    }

    public static Iterable<Integer> pathTo(Graph g, int u, int v){
        BFS(g, u);
        return pathTo(v);
    }

    public static Iterable<Integer> pathTo(Graph g, Node A, Node B){
        BFS(g, A.ID());
        return pathTo(B.ID());
    }

//================( Connected Component )=============================\\

    private static int[] id;
    private static int count;

    public static int[] connComp(Graph G){
        marked = new boolean[G.V()];
        id = new int[G.V()];
        count = 1;

        for(int i=0; i<G.V(); i++){
            if(!marked[i]) {
                BFS(G, i);
                for (int j=0; j<G.V(); j++){
                    if (hasPathTo(j))
                        id[j] = count;
                }
                count++;
            }
        }
        return id;
    }

    public static boolean connected(int u, int v){
        return (id[u] == id[v]);
    }

    public static int count(){
        return count;
    }

//================( Reverse Graph )=============================\\

    public static Graph Reverse(Graph g){
        if(g.isDigraph()){
            ArrayList<Edge> revEdges = new ArrayList<>(g.E());

            for(Edge e: g.edges){
                Edge re = new Edge(e.w(), e.v(), e.weight());
                revEdges.add(re);
            }

            return new Graph(g.nodes, revEdges, true);
        }
        else return g;
    }

//================( Copy Graph )=============================\\

    public static Graph Copy(Graph g){
        return (new Graph(g));
    }

//================( Cycle Detection )=============================\\

    private static Stack<Integer> cycle;
    private static boolean[] onStack;

    private static void dfsCycleU(Graph G, int u, int v){
        marked[v] = true;
        for (Edge e : G.adj(v)) {
            if (cycle != null) return;
            if (!marked[e.end(v)]) {
                edgeTo[e.end(v)] = v;
                dfsCycleU(G, v, e.end(v));
            }
            else if (e.end(v) != u) {
                cycle = new Stack<>();
                for (int x = v; x != e.end(v); x = edgeTo[x])
                    cycle.push(x);
                cycle.push(e.end(v));
                cycle.push(v);
            }
        }

    }

    public static boolean hasCycle(Graph G){
        marked = new boolean[G.V()];
        edgeTo = new int[G.V()];
        onStack = new boolean[G.V()];
        if(G.isDigraph()){
            if(G.nodes.size()>0) {
                for(int u=0; u<G.V(); u++)
                    if(!marked[u])
                     dfsCycleD(G, u);
            }
        }else{
            if(G.nodes.size()>2) {
                for(int u=0; u<G.V(); u++)
                    if(!marked[u])
                        dfsCycleU(G, -1, u);
            }
        }
        return (cycle != null);
    }

    private static void dfsCycleD(Graph G, int s){
        Stack<Integer> stack = new Stack<>();

        marked[s] = true;
        onStack[s] = true;
        for(Edge e: G.adj(s)){
            if(cycle != null) return;
            else if(!marked[e.w()]){
                edgeTo[e.w()] = e.v();
                dfsCycleD(G, e.w());
            }
            else if(onStack[e.w()]){
                cycle = new Stack<>();
                for(int x=e.v(); x!=e.w(); x=edgeTo[x])
                    cycle.push(x);
               cycle.push(e.w());
               cycle.push(e.v());
            }
            onStack[s] = false;
        }
    }

    public static Iterable<Integer> cycle(Graph g){
        if(hasCycle(g))
            return cycle;
        else
            return null;
    }

//================( Topological Sorting )=============================\\
    private static Queue<Integer> pre;
    private static Queue<Integer> post;
    private static Stack<Integer> reversePost;

    public static Iterable<Integer> ToplogicalOrder(Graph G){
        if(hasCycle(G)) return null;

        pre = new LinkedList<>();
        post = new LinkedList<>();
        reversePost = new Stack<>();
        marked = new boolean[G.V()];

        for(int v=0; v<G.V(); v++) {
            if(!marked[v])
                dfsTopOrder(G, v);
        }
        return reversePost;
    }

    private static void dfsTopOrder(Graph G, int s) {
        pre.add(s);
        marked[s] = true;
        for(Edge e: G.adj(s))
            if(!marked[e.w()])
                dfsTopOrder(G, e.w());
        post.add(s);
        reversePost.push(s);
    }

    public Iterable<Integer> preorder(){return pre;}
    public Iterable<Integer> postorder() {return post;}
    public Iterable<Integer> reversePostorder() {return reversePost;}

//================( Minimum Spanning Tree - Kruskal )=============================\\

    /* this 2 lines can be used instead of g.edges.sort() inside the method:
    PriorityQueue<Edge> pq = new PriorityQueue<>(g.E(), Comparator.comparingDouble(obj -> obj.weight()));
        pq.addAll(g.edges);
    */

    public static ArrayList<Edge> MST(Graph g){
        if(g.isDigraph()) return null;

        ArrayList<Edge> mst = new ArrayList<>(g.V()-1);

        Collections.sort(g.edges);

        int[] parent = new int[g.V()];
        for(int i=0; i<g.V(); i++)
            parent[i] = i;

        for(int index=0; index<g.E(); index++){
            Edge edge = g.edges.get(index);

            int x_set = findmst(parent, edge.v());
            int y_set = findmst(parent, edge.w());

            if(x_set != y_set) {
                mst.add(edge);
                unionmst(parent, x_set, y_set);
            }
        }
           return mst;
    }

    private static int findmst(int[] parent, int vertex){
        if(parent[vertex] != vertex)
            return findmst(parent, parent[vertex]);
        return vertex;
    }

    private static void unionmst(int[] parent, int x, int y){
        int x_set_parent = findmst(parent, x);
        int y_set_parent = findmst(parent, y);
        parent[y_set_parent] = x_set_parent;
    }

//================( Shortest Path - Dijkstra )=============================\\

    private static Node[] distTo;
    private static PriorityQueue<Node> pq;
   // private static Node distto[];

    public static void ShortestPath(Graph G, Node n){ShortestPath(G, n.ID());}

    public static Iterable<Integer> shortestPathTo(Graph G, int from, int to){
        if(hasNegEdge){
            ShortestPath(G, from);
            return shortestPathTo(to);
        }
        ShortestPathDijkstra(G, from);
        return shortestPathToDijkstra(to);
    }

    public static void ShortestPathDijkstra(Graph G, int s){
        edgeTo = new int[G.V()];
        distTo = new Node[G.V()];
        pq = new PriorityQueue<>(G.V(), Comparator.comparingDouble(obj -> (Double)obj.Value()));
        source = s;

        for(int v=0; v<G.V(); v++) {
            Double dist = Double.POSITIVE_INFINITY;
            distTo[v] = new Node(v, dist);
        }
        distTo[s].setValue(0.0);

        pq.add(distTo[s]);
        while(!pq.isEmpty())
            relax(G, pq.poll());
    }

    private static void relax(Graph G, Node v){
        for(Edge e: G.adj(v.ID())){
            int w = e.end(v.ID());
            if((Double)distTo[w].Value() > (Double) distTo[v.ID()].Value() + e.weight()){
                distTo[w].setValue((Double) distTo[v.ID()].Value() + e.weight());
                edgeTo[w] = e.start();
                if(!pq.contains(distTo[w])) pq.add(distTo[w]);
            }
        }
    }

    public static double distToDijkstra(int v) {return (double) distTo[v].Value();}
    public static boolean hasShortestPathToDijkstra(int v) {return ((double) distTo[v].Value()) < Double.POSITIVE_INFINITY;}
    public static Iterable<Integer> shortestPathToDijkstra(int v){
        if(!hasShortestPathToDijkstra(v)) return null;
        Stack<Integer> path = new Stack<>();
        for(int x=v; x!=source; x=edgeTo[x])
            path.push(x);
        path.push(source);
        ArrayList<Integer> list = new ArrayList<>(path);
        Collections.reverse(list);
        return list;
    }

//================( Shortest Path - Bellman-Ford )=============================\\

    private static boolean hasNegWtCycle;
    private static boolean hasNegEdge;
    private static double[] dist;

    private static boolean hasNegEdge(Graph G){
        for(int i=0; i<G.E(); i++)
            if(G.edges.get(i).weight() < 0.0)
                return true;
        return false;
    }

    public static void ShortestPath(Graph G, int S){
        hasNegEdge = hasNegEdge(G);
        if(!hasNegEdge) {
            ShortestPathDijkstra(G, S);
            return;
        }
        hasNegWtCycle = false;
        source = S;
        dist = new double[G.V()];
        for(int v=0; v<G.V(); v++)
            dist[v] = Double.POSITIVE_INFINITY;
        dist[source] = 0.0;

        for(int i=1; i<G.V(); i++){
            for(int j=0; j<G.E(); j++){
                int u = G.edges.get(j).start();
                int v = G.edges.get(j).end(u);
                float wt = G.edges.get(j).weight();

                if((dist[u] != Double.POSITIVE_INFINITY) && (dist[u] + wt) < dist[v]) {
                    dist[v] = dist[u] + wt;
                    edgeTo[v] = u;
                }
            }
        }

        for(int j=0; j<G.E(); j++){
            int u = G.edges.get(j).start();
            int v = G.edges.get(j).end(u);
            float wt = G.edges.get(j).weight();

            if((dist[u] != Double.POSITIVE_INFINITY) && (dist[u] + wt) < dist[v]) {
                hasNegWtCycle = true;
                return;
            }
        }
    }

    public static double distTo(int v) {
        if(!hasNegEdge)
            return distToDijkstra(v);
        return dist[v];
    }

    public static boolean hasShortestPathTo(int v){
        if(!hasNegEdge)
            return hasShortestPathToDijkstra(v);
        return (dist[v] < Double.POSITIVE_INFINITY);
    }

    public static Iterable<Integer> shortestPathTo(int v){
        if(!hasNegEdge)
            if(!hasShortestPathToDijkstra(v)) return null;
        if(hasNegEdge)
            if(!hasShortestPathTo(v)) return null;

        Stack<Integer> path = new Stack<>();
        for(int x=v; x!=source; x=edgeTo[x])
            path.push(x);
        path.push(source);
        ArrayList<Integer> list = new ArrayList<>(path);
        Collections.reverse(list);
        return list;
    }

//================( Max Flow ~ Min Cut ~ EK:FF)=============================\\

    public static long MaxFlow(Graph G, int S, int T){
        if(S == T) return 0;

        Graph RG = new Graph(G);
        Graph revG = Reverse(G);
        for(Edge e: revG.edges){
            e.setWeight((float)0.0);
            RG.addEdge(e);
        }

        edgeTo = new int[G.V()];
        marked = new boolean[G.V()];

        double maxflow = 0.0;

        while(hasPathTo(RG, S, T)){
            double pathflow = Double.MAX_VALUE;

            for(int v=T; v!=S; v=edgeTo[v]) {
                int u = edgeTo[v];
                for(Edge e: RG.adj(u))
                    if(e.w() == v)
                pathflow = Math.min(pathflow, e.weight());
            }

            for(int v=T; v!=S; v=edgeTo[v]) {
                int u = edgeTo[v];
                for(Edge e: RG.adj(u))
                    if(e.w() == v) {
                        float wt = e.weight();
                        e.setWeight(wt-(float)pathflow);
                    }
                for(Edge e: RG.adj(v))
                    if(e.end(v) == u){
                        float wt = e.weight();
                        e.setWeight(wt+(float)pathflow);
                    }
            }

            maxflow += pathflow;
            for(Edge e: RG.edges) if(e.weight()<=0.0) RG.delEdge(e);
        }
        return (long) maxflow;
    }

}
