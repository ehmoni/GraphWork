
public class Edge implements Comparable<Edge>{

    private String EdgeID;
    private int v;
    private int w;
    private float weight;

    Edge(int v, int w){
        this.v = v;
        this.w = w;
        this.weight = (float) 0.0;
        EdgeID = v + "-" + w;
    }

    Edge(int v, int w, float weight){
        this(v, w);
        this.weight = weight;
    }

    Edge(int v, int w, float weight, String id){
        this(v, w, weight);
        EdgeID = id;
    }

    Edge(Edge e){
        this(e.v, e.w, e.weight, e.EdgeID);
    }

    Edge(Node A, Node B){
        this(A.ID(), B.ID());
    }

    Edge(Node A, Node B, float weight){
        this(A.ID(), B.ID(), weight);
    }

    Edge(Node A, Node B, float weight, String id){
        this(A.ID(), B.ID(), weight, id);
    }

    public void setID(String id) {EdgeID = id;}
    public void setWeight(float wt) {weight = wt;}

    public int v() {return v;}
    public int w() {return w;}
    public float weight() {return weight;}
    public String edgeID() {return EdgeID;}

    public int start() {return v;}

    public int end(int vertex){
        if (vertex == v) return w;
        else if(vertex == w) return v;
        else throw new IllegalArgumentException();
    }

    public int compareTo(Edge that){
        if(this.weight < that.weight) return -1;
        else if(this.weight > that.weight) return 1;
        else return 0;
    }

    public String toString(){
        return String.format("(%s): %.2f", EdgeID, weight);
    }

}

