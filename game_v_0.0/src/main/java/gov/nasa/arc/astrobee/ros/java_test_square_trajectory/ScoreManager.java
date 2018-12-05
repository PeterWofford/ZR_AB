package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

import java.io.*;

public class ScoreManager {

    public static int fwh;
    public static int pdh;
    public static int arh;
    public static int hsh;
    public static int fwd;
    public static int pdd;
    public static int hsd;
    public static int asd;
    public static int mp1;
    public static int mp2;
    public static int hrn;
    public static int hrd;
    public static int ts;
    public static boolean hit;

    public ScoreManager(){
        fwh = 0; //c
        pdh = 0; //c
        arh = 0; //c
        hsh = 0; //c
        fwd = 0; //c
        pdd = 0; //c
        hsd = 0; //c
        asd = 0; //c
        mp1 = 0; //c
        mp2 = 0; //c
        hrn = 0;
        hrd = 0;
        ts = 0;
        boolean hit = false;
    }

    public static void updatem(int mp1change, int mp2change){
        System.out.println("no hit");
        hit = false;
        mp1 += mp1change;
        mp2 += mp2change;
    }

    public static void updateh(boolean don, String pollen){
        hit = true;
        if(pollen.equals("FIREWEED")){
            if(don){
               fwd += 1;
            }else{
               fwh += 1;
            }
        }else if(pollen.equals("HONEYSUCKLE")){
            if(don){
               hsd += 1;
            }else{
               hsh += 1;
            }
        }else if(pollen.equals("ARUGULA")){
            if(don){
               asd += 1;
            }else{
               arh += 1;
            }
        }else if(pollen.equals("PANDORA")){
            if(don){
               pdd += 1;
            }else{
               pdh += 1;
            }
        }else{
            System.out.println("Invalid pollen");
        }
    }

    public static void updateratio(int score, int num, int den){
        ts = score;
        hrn = num;
        hrd = den;
    }

    public static void initial(){
        try {
            FileWriter writer = new FileWriter("/home/" + System.getProperty("user.name") + "/ZR_AB/out.txt");
            BufferedWriter out = new BufferedWriter(writer);
            out.write("---------");
            out.newLine();

            out.write(Integer.toString(0)); //fw hits
            out.newLine();
            out.write(Integer.toString(0)); //pd hits
            out.newLine();
            out.write(Integer.toString(0)); //ar hits
            out.newLine();
            out.write(Integer.toString(0)); //hs hits
            out.newLine();
            out.write(Integer.toString(0)); //fw d
            out.newLine();
            out.write(Integer.toString(0)); //pd d
            out.newLine();
            out.write(Integer.toString(0)); //ar d
            out.newLine();
            out.write(Integer.toString(0)); //hs d
            out.newLine();
            out.write(Integer.toString(0)); //mp1
            out.newLine();
            out.write(Integer.toString(0)); //mp2
            out.newLine();
            out.write(Integer.toString(0)); //fr1
            out.newLine();
            out.write(Integer.toString(0)); //fr2
            out.newLine();
            out.write(Integer.toString(0)); //hrn
            out.newLine();
            out.write(Integer.toString(0)); //hrd
            out.newLine();
            out.write(Integer.toString(0)); //total score
            out.newLine();

            out.close();
        }
        catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public static void updategui(){
        try {
            FileWriter writer = new FileWriter("/home/" + System.getProperty("user.name") + "/ZR_Astrobee/out.txt");
            BufferedWriter out = new BufferedWriter(writer);

            if(hit){
                out.write("SCORE!");
                out.newLine();
            }else{
                out.write("MISS!");
                out.newLine();
            }
            //out.write(sc);
            //out.newLine();

            out.write(Integer.toString(fwh)); //fw hits
            out.newLine();
            out.write(Integer.toString(pdh)); //pd hits
            out.newLine();
            out.write(Integer.toString(arh)); //ar hits
            out.newLine();
            out.write(Integer.toString(hsh)); //hs hits
            out.newLine();
            out.write(Integer.toString(fwd)); //fw d
            out.newLine();
            out.write(Integer.toString(pdd)); //pd d
            out.newLine();
            out.write(Integer.toString(asd)); //ar d
            out.newLine();
            out.write(Integer.toString(hsd)); //hs d
            out.newLine();
            out.write(Integer.toString(mp1)); //mp1
            out.newLine();
            out.write(Integer.toString(mp2)); //mp2
            out.newLine();
            out.write(Integer.toString(0)); //fr1
            out.newLine();
            out.write(Integer.toString(0)); //fr2
            out.newLine();
            out.write(Integer.toString(hrn)); //hrn
            out.newLine();
            out.write(Integer.toString(hrd)); //hrd
            out.newLine();
            out.write(Integer.toString(ts)); //total score
            out.newLine();

            out.close();
        }
        catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }


    public static void main(String... args) throws IOException {
        try {
            FileWriter writer = new FileWriter("/home/" + System.getProperty("user.name") + "/ZR_Astrobee/out.txt");
            BufferedWriter out = new BufferedWriter(writer);
            out.write("SCORE!");
            out.newLine();

            out.write(Integer.toString(fwh)); //fw hits
            out.newLine();
            out.write(Integer.toString(pdh)); //pd hits
            out.newLine();
            out.write(Integer.toString(arh)); //ar hits
            out.newLine();
            out.write(Integer.toString(hsh)); //hs hits
            out.newLine();
            out.write(Integer.toString(fwd)); //fw d
            out.newLine();
            out.write(Integer.toString(pdd)); //pd d
            out.newLine();
            out.write(Integer.toString(asd)); //ar d
            out.newLine();
            out.write(Integer.toString(hsd)); //hs d
            out.newLine();
            out.write(Integer.toString(mp1)); //mp1
            out.newLine();
            out.write(Integer.toString(mp2)); //mp2
            out.newLine();
            out.write(Integer.toString(0)); //fr1
            out.newLine();
            out.write(Integer.toString(0)); //fr2
            out.newLine();
            out.write(Integer.toString(hrn)); //hrn
            out.newLine();
            out.write(Integer.toString(hrd)); //hrd
            out.newLine();
            out.write(Integer.toString(ts)); //total score
            out.newLine();

            out.close();
        }
        catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
