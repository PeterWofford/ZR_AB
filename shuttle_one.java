import java.io.*;

public class shuttle_one {

/**
 * @param args
 * @throws IOException 

 */

    


    public static void main(String[] args) throws IOException {
        //******CHANGE THESE VARIABLES FOR IT TO INSERT******
        String textInsertPlaceHolder = "public class PlayerTemplate {";		// replace with the main header
        String studentCode = "/home/peterwof/ZR_AB_fakeDB/API_fields.txt";// path to api fields code
        String javaGameCodeFile = "/home/peterwof/ZR_AB_fakeDB/PlayerTemplate.java";//path to template
	    String pathToCombinedFile = "/home/peterwof/ZR_AB_fakeDB/PlayerTemplate2.java"; //path to save the combined student and template code

        try {

            //set up reading streams and writing file
            BufferedReader br = new BufferedReader(new FileReader(studentCode));
            BufferedReader br2 = new BufferedReader(new FileReader(javaGameCodeFile));
            PrintWriter writer=new PrintWriter(pathToCombinedFile);
            
            

            //go through both files and write to new one combining them
            String reader1="";
            String reader2="";
            while ((reader2=(br2.readLine()))!=null){
                writer.write(reader2+"\n");
                if (reader2.equals(textInsertPlaceHolder)){
                    while((reader1=(br.readLine()))!=null){
                        writer.write(reader1+"\n");
                    }
                } 
                
            }

            //close the files
            br.close();
            br2.close();
            writer.close();
            

            //Run the file
            try{
                //Compile
                // Process theProcess = Runtime.getRuntime().exec("javaC Combined.java");
                // theProcess.waitFor();
                
                //Run
                //Process theProcess2 = Runtime.getRuntime().exec("java Combined");
                //String line = null;

                //Comment out the following 5 lines if there is not output to read from running the java file
                // this just outputs the threads output stream to this programs output stream.
                
                //BufferedReader in = new BufferedReader( new InputStreamReader(theProcess2.getInputStream()));
                //while ((line = in.readLine()) != null){
                //    System.out.println(line);
                //}
                ///theProcess2.waitFor(); 
                
                //Delete intermediate files
                //Process theProcess3 = Runtime.getRuntime().exec("rm Combined.class && rm Combined.java");
                //theProcess3.waitFor();
            }catch(Exception e){
                e.printStackTrace();  
            }

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        System.out.println("done moving");

    }
}
