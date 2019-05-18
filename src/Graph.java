import java.util.ArrayList;
import java.util.Arrays;


public class Graph {
    private int V;
    private int E;
    private boolean Digraph;
    public ArrayList<Node> nodes;
    private ArrayList<Edge>[] adj;
    public ArrayList<Edge> edges;

    Graph(int capacity){
        Digraph = false;
        V = capacity;
        E = 0;
        nodes = new ArrayList<>(V);
        edges = new ArrayList<>(V);
        adj = (ArrayList<Edge>[]) new ArrayList[V];
        for(int v=0;v<V;v++) {
            nodes.add(new Node(v));
            adj[v] = new ArrayList<>();
        }
    }

    Graph(int V, boolean digraph){
        this(V);
        Digraph = digraph;
    }

    Graph(int V, ArrayList<Edge> Edges, boolean digraph){
        this(V,digraph);
        E = Edges.size();
        edges = new ArrayList<>(Edges.size());
        for(Edge edge: Edges)
            edges.add(new Edge(edge.v(), edge.w(), edge.weight()));
        edges.forEach((e) -> adj[e.v()].add(e));
        if(!Digraph){edges.forEach((e) -> adj[e.w()].add(e));}
    }

    Graph(ArrayList<Node> Nodes, ArrayList<Edge> Edges, boolean digraph){
        this(Nodes.size(), Edges, digraph);
        nodes = (ArrayList<Node>) Nodes.clone();
    }

    Graph(Graph G){
        this(G.nodes, G.edges, G.isDigraph());
    }

    public int V() {return V;}
    public int E() {return E;}
    public boolean isDigraph() {return Digraph;}

    public void addNode(Node node){
        if(node.ID()==V){
            ArrayList<Edge>[] temp = Arrays.copyOf(adj, V+1);
            adj = temp;
            nodes.add(node);
            V++;
        }
        else
            throw new IllegalArgumentException();
    }

    public void addNode(int node){
        Node newNode = new Node(node);
        addNode(newNode);
    }

    public void delEdge(int v, int w) {
        /*for(Edge e: adj(v))
            if(e.w() == w)
                adj[v].remove(e);*/
        adj[v].removeIf(e -> e.w() == w);
        refreshGraph();
    }

    public void delEdge(Edge edge){
        delEdge(edge.v(), edge.w());
    }

    public void delEdge(String edgeid){
        for(int v=0; v<adj.length; v++)
            for(Edge e: adj(v))
                if(e.edgeID().equals(edgeid))
                    adj[v].remove(e);
        refreshGraph();
    }

    public void addEdge(Node v, Node w, float weight){
        Edge e = new Edge(v, w, weight);
        edges.add(e);
        E++;

        adj[e.v()].add(e);
        if(!Digraph) adj[e.w()].add(e);
    }

    public void addEdge(Node v, Node w){
        addEdge(v, w, (float) 0.0);
    }

    public void addEdge(int v, int w, float weight){
        Node A = new Node(v);
        Node B = new Node(w);
        addEdge(A, B, weight);
    }

    public void addEdge(int v, int w){
        addEdge(v, w, (float)0.0);
    }

    public void addEdge(Edge e){
        addEdge(e.v(), e.w(), e.weight());
    }

    public void refreshGraph(){
        edges = new ArrayList<>(E);
        for(int i=0; i<adj.length; i++){
            for(int j=0; j<adj[i].size(); j++){
                if(!edges.contains(adj[i].get(j)))
                    edges.add(adj[i].get(j));
            }
        }
    }

    public Iterable<Edge> adj(int v){
        return adj[v];
    }

    public String toString(){
        String str = "";
        for(int i=0; i<adj.length; i++)
            str += i + adj[i].toString() + "\n";
        return str;
    }

}
