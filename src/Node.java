
public class Node{

    private int NodeID;
    private Object obj;

    Node(){
        NodeID = -1;
        obj = new Object();
        obj = null;
    }

    Node(int id){
        this();
        NodeID = id;
    }

    Node(int id, Object o){
        this();
        NodeID = id;
        obj = o;
    }

    public int ID() {return NodeID;}
    public Object Value() {return obj;}
    public void setValue(Object Obj) {this.obj = Obj;}

    public String toString(){
        if(obj == null)
            return String.format("(%d) ", NodeID);
        else
            return String.format("(%d): %s", NodeID, obj.toString());
    }

}