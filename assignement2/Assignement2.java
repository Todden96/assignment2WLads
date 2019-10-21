/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package assignement2;

import ilog.concert.IloException;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import Benders.MasterProblem;
import java.util.Locale;

/**
 *
 * @author vtodd
 */
public class Assignement2 {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) throws FileNotFoundException, IloException{
   
        // 1. READS THE GENERATORS DATA
        // The generators data are stored in the generators.txt file
        // Note that the generators.txt file must be in project's specific NetBeans 
        // directory. If the generators.txt file is elsewhere you will need to provide
        // an absolute path such as "C://Users/Documents/MyFolder/generators.txt"
        File generatorsFile = new File("generators.txt");
        Scanner scanner = new Scanner(generatorsFile).useLocale(Locale.US);
        
        // We skip the first two lines of the file since they are a header
        // That is, we move the cursor past the first two lines (at the beginning of the third)
        scanner.nextLine();
        scanner.nextLine();
        
        // We create the arrays for storing the data we read.
        // These arrays will then be used to create an instance of 
        // the UnitCommitmentProblem class.
        
        // We have 31 generators in the file
        int nGenerators = 31;
        String[] names = new String[nGenerators];
        double[] minProduction = new double[nGenerators];
        double[] maxProduction = new double[nGenerators];
        double[] startUpCosts = new double[nGenerators];
        double[] commitmentCosts = new double[nGenerators];
        double[] rampUp = new double[nGenerators];
        double[] rampDown = new double[nGenerators];
        int[] minUpTime = new int[nGenerators];
        int[] minDownTime = new int[nGenerators];
        double[] productionCost = new double[nGenerators];
        
        for(int i = 1; i <= nGenerators; i++){
            names[i-1] = scanner.next();
            
            minProduction[i-1] = scanner.nextDouble();
            
            maxProduction[i-1] = scanner.nextDouble();
            
            startUpCosts[i-1] = scanner.nextDouble();
            
            commitmentCosts[i-1] = scanner.nextDouble();
            
            rampUp[i-1] = scanner.nextDouble();
            
            // RampDown is equal to RampUp
            rampDown[i-1] = rampUp[i-1];
            minUpTime[i-1] = scanner.nextInt();
            
            minDownTime[i-1] = scanner.nextInt();
            
            productionCost[i-1] = scanner.nextDouble();
            

        }
        
        
        // 2. READS THE LOADS DATA
        // We have 24 hours
        int nHours = 24;
        double[] demand = new double[nHours];
        
        // *************
        File loadsFile = new File("loads.txt");
        Scanner scanner2 = new Scanner(loadsFile);
        
        scanner2.nextLine();
        for(int i = 1; i <= nHours; i++){
            demand[i-1] = scanner2.nextDouble();
        }
        
        
        
        // We calculate min up time and down time for each t
        int[][] minUpTimeAtT = new int[nGenerators][nHours];
        int[][] minDownTimeAtT = new int[nGenerators][nHours];
        for(int g = 1; g <= nGenerators; g++){
            for(int t = 1; t <= nHours; t++){
                minUpTimeAtT[g-1][t-1] = Math.min(t+minUpTime[g-1]-1, nHours);
                minUpTimeAtT[g-1][t-1] = Math.min(t+minDownTime[g-1]-1, nHours);
            }
        }
        
        double[] sheddingCosts = new double[nHours];
        
        for(int t = 1; t <=nHours; t++){
            sheddingCosts[t-1] = 29.58*2;
                    //Math.max(productionCost(),0); TBC
        }
        // ***************
        // TO BE COMPLETED:
        // Calculate the highest production cost, and set the shedding cost
        // twice as high as the highest production cost
        // .......
        // .......
        // *********************
        
        
        
        // 3. CREATES AN INSTANCE OF THE UNIT COMMITMENT PROBLEM
        
        UnitCommitmentProblem ucp 
                = new UnitCommitmentProblem(nHours,nGenerators,
                        names,commitmentCosts,productionCost,
                        sheddingCosts, startUpCosts,
                        minUpTime,minDownTime,
                        minUpTimeAtT,minDownTimeAtT,
                        minProduction,maxProduction,
                        rampUp,rampDown,
                        demand
        );
        
        // 4. CREATES THE MATHEMATICAL MODEL AND SOLVES IT (WITHOUT DECOMPOSITION)
        UnitCommitmentModel ucpm = new UnitCommitmentModel(ucp);
        ucpm.solve();
        ucpm.printSolution();
        
        
        // 5. SOLVES THE PROBLEM USING BENDERS DECOMPOSITION
        // ***************
        MasterProblem mp = new MasterProblem(ucp);
        mp.solve();
        System.out.println("Best value with integer Benders = "+mp.getObjective());
        //mp.printSolution();
        
        
        // TO BE COMPLETED:
        // In order to complete this code you might need to create the classes
        // MasterProblem, OptimalitySubProblem, and FeasibilitySubProblem
        // .......
        // .......
        // *********************
        
        
        
    
    }
    
}
