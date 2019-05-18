import java.io.File;
import java.io.FileNotFoundException;
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;

public class Tester {
//-----------------------Input File--------------------------------------------------//

    public static void main(String[] args){
        File file = new File("C:\\Java\\JavaTutorial\\Inputs\\FFC.txt");
        Scanner sc = null;
        try {
            sc = new Scanner(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

//-----------------------Graph Ready--------------------------------------------------//
        Graph g = new Graph(sc.nextInt(), true);
        int E = sc.nextInt();
        while (sc.hasNextInt()) {
            int u = sc.nextInt();
            int v = sc.nextInt();
            float w = sc.nextFloat();
            g.addEdge(u, v, w);
        }
        System.out.println("\nGraph :-  \n" + g.toString());
////////////////////////////////////////////////////////////////////////////////////////

    /*     Graphwork.BFS(g, 0);
        System.out.println(Graphwork.pathTo(12));

        System.out.println(g.edges);
        Graph rG = Graphwork.Reverse(g);
        System.out.println(rG.edges);
        int[] x = Graphwork.connComp(g);
        for(int i=0; i<g.V(); i++)
            System.out.println(i + ": " + x[i]);

        int a = 3, b = 7;
        System.out.println("Connected: (" + a + "," + b + ")" + Graphwork.connected(a, b));


        System.out.println(Graphwork.hasPathTo(g, 7,3));*/

        //System.out.println(Graphwork.hasCycle(g));
        //System.out.println(Graphwork.cycle(g));

        //Graphwork.DFS(g, 0);
        //System.out.println(Graphwork.pathTo(4));

        //System.out.println(Graphwork.ToplogicalOrder(g));

        System.out.println(Gwork.MaxFlow(g, 0, 5));

        //System.out.println(Gwork.shortestPathTo(6));

        //System.out.println(Gwork.shortestPathTo(g, 4, 6));


        /*Integer x = 15;
        Node nd = new Node(1, x);
        Node md = new Node(0, x+5);
        Edge e = new Edge(0,1, 4.5);

        Graph g = new Graph(2);
        g.addEdge(e);

        System.out.println("Node1 :-  " + nd.toString());
        System.out.println("Node2 :-  " + md.toString());
        System.out.println("Edges :-  " + e.toString());
        System.out.println("\n\nGraph :-  \n" + g.toString());*/
    }
}
