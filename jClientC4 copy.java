/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2015 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


/*
    Objetivos a atingir:
    Corrigir o MakePath do C3
    --erro está na função AddCaminho

    
*/

import java.beans.PersistenceDelegate;
import java.io.*;
import java.net.*;
import java.util.*;

import org.xml.sax.*;
import org.xml.sax.helpers.DefaultHandler;
import javax.xml.parsers.SAXParserFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;


import ciberIF.*;

/**
 *  the map
 */
class Map {
    static final int CELLROWS = 7;
    static final int CELLCOLS = 14;

    /*! In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
     *  to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
     */
	char[][] labMap;

    public Map()
    {
	    labMap = new char[CELLROWS*2-1][CELLCOLS*2-1];

        for(int r=0; r < labMap.length; r++) {
            Arrays.fill(labMap[r],' ');
        }
    }

};

/**
 *  class MapHandler parses a XML file defining the labyrinth
 */
class MapHandler extends DefaultHandler {

	/**
	 */
	private Map map;

	/**
	 * returns the Parameters collected during parsing of message
	 */
	Map getMap()
	{
		return map;
	}

	public void startElement(String namespaceURI,
	                         String sName, // simple name
	                         String qName, // qualified name
	                         Attributes attrs)
	throws SAXException
	{

	    //Create map object to hold map
	    if(map == null) map = new Map();

		if(qName.equals("Row")) {  // Row Values

            if (attrs != null) {
                String rowStr=attrs.getValue("Pos");
                if(rowStr!=null) {
                    int row = Integer.valueOf(rowStr).intValue();
		            String pattern = attrs.getValue("Pattern");
                    for(int col=0; col < pattern.length(); col++) {
                       if(row % 2 == 0) { // only vertical walls are allowed here
                            if(pattern.charAt(col)=='|') {
                               map.labMap[row][(col+1)/3*2-1] = '|';
                            }
                       }
                       else {// only horizontal walls are allowed at odd rows
                           if(col % 3 == 0) { // if there is a wall at this collumn then there must also be a wall in the next one
                               if(pattern.charAt(col)=='-') {
                                  map.labMap[row][col/3*2] = '-';
                               }
                           }
                       }
                    }
                }
            }
        }
	}

	public void endElement(String namespaceURI,
			        String sName, // simple name
			        String qName  // qualified name
						        )
	throws SAXException
	{
	}
};


/**
 * example of a basic agent
 * implemented using the java interface library.
 */
public class jClientC4 {

    ciberIF cif;
    Map map;
    enum State {GA, RL, RR, INV, END}

    public static void main(String[] args) throws IOException{

        String host, robName;
        int pos;
        int arg;
        Map map;


        //default values
        host = "localhost";
        robName = "jClientC4";
        pos = 1;
        map = null;

        // parse command-line arguments
        try {
            arg = 0;
            while (arg<args.length) {
                if(args[arg].equals("--pos") || args[arg].equals("-p")) {
                        if(args.length > arg+1) {
                                pos = Integer.valueOf(args[arg+1]).intValue();
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--robname") || args[arg].equals("-r")) {
                        if(args.length > arg+1) {
                                robName = args[arg+1];
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--host") || args[arg].equals("-h")) {
                        if(args.length > arg+1) {
                                host = args[arg+1];
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--map") || args[arg].equals("-m")) {
                        if(args.length > arg+1) {

                                MapHandler mapHandler = new MapHandler();

                                SAXParserFactory factory = SAXParserFactory.newInstance();
                                SAXParser saxParser = factory.newSAXParser();
                                FileInputStream fstream=new FileInputStream(args[arg+1]);
                                saxParser.parse( fstream, mapHandler );

                                map = mapHandler.getMap();

                                arg += 2;
                        }
                }
                else throw new Exception();
            }
        }
        catch (Exception e) {
                print_usage();
                return;
        }

        // create client
        jClientC4 client = new jClientC4();

        client.robName = robName;
        double[] angles = {0,90,-90,180};
        // register robot in simulator
        client.cif.InitRobot2(robName, pos, angles, host);
        client.map = map;
        client.printMap();

        // main loop
        client.mainLoop();

    }

    // Constructor
    jClientC4() {
            cif = new ciberIF();
            beacon = new beaconMeasure();
            beaconToFollow = 0;
            ground=-1;
    }

    /**
     * reads a new message, decides what to do and sends action to simulator
     */
    public void mainLoop () throws IOException{ //ver aqui.....
        cif.ReadSensors();
        x0=cif.GetX();
        y0=cif.GetY();
        init=true;
        fillMap();
        nbeacons = cif.GetNumberOfBeacons();
        objetivos = new vetor[nbeacons]; 
        vetor aux = new vetor ();
        while(true) {
            cif.ReadSensors(); 
            decide();
        }
    }


     /*! In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
     *  to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
     */

    /**
     * basic reactive decision algorithm, decides action based on current sensor values
     */
    public void decide() throws IOException{ // num ciclo decide o que vai fazer -> drivemotors
            if(cif.IsObstacleReady(0))
                    irSensor0 = cif.GetObstacleSensor(0);
            if(cif.IsObstacleReady(1))
                    irSensor1 = cif.GetObstacleSensor(1);
            if(cif.IsObstacleReady(2))
                    irSensor2 = cif.GetObstacleSensor(2);
            if(cif.IsObstacleReady(3))
                    irSensor3 = cif.GetObstacleSensor(3);
            if(cif.IsCompassReady())
                    compass = cif.GetCompassSensor();
            if(cif.IsGroundReady())
                    ground = cif.GetGroundSensor(); // ver se esta num dos alvos
            if(cif.IsGPSReady()){
                    x = cif.GetX() - x0;//+-22
                    y = cif.GetY() - y0;//+-13
            }

            if(cif.IsBeaconReady(beaconToFollow))
                    beacon = cif.GetBeaconSensor(beaconToFollow);

            if(targetReached()){
                System.out.println("\n Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + " ir3="+ irSensor3+"\n" + "bussola=" + compass + " GPS-X=" + x + " GPS-y=" + y);
                System.out.println("f: "+ParedeFrente()+" esq: "+ParedeEsquerda()+" dir: "+ParedeDireita()+" tras: "+ParedeTras());
                System.out.println("init: "+init+" targetReached? "+targetReached() + "\n");
            }
            //funcao geral: detetar se está no centro, se sim corre mapping e calcular next, se nao manda andar para onde é preciso
            //andar implica ou curvas ou ahead
            state = Estados();
            switch(state) { 

                case GA: 
                    if(targetReached()){
                        if(caminho.isEmpty()) mappingDecode();
                        else runCaminho();
                        cif.DriveMotors(0.0,0.0);
                    }
                    else{
                        goAhead();
                    }
                    break;

                case RL:
                    if(targetReached()){
                        if(caminho.isEmpty()) mappingDecode();
                        else runCaminho();
                        cif.DriveMotors(0.0,0.0);
                    }
                    else{
                        goLeft(); 
                    }
                    break;

                case RR:
                    if(targetReached()){ 
                        if(caminho.isEmpty()) mappingDecode();
                        else runCaminho();
                        cif.DriveMotors(0.0,0.0);
                    }
                    else{
                        goRight(); 
                    }
                    break;

                case INV:
                    goInv(); 
                    break;

                case END:
                   
                    if(cif.GetFinished()){ //verificar se deu finish
                        System.exit(0);
                    }
                    cif.DriveMotors(0.0,0.0);
                    writeMap();
                    writeCaminho(); //->nao funciona
                    cif.Finish();
                   
                    break;
            }
            return;
    }

    public State Estados() throws IOException{
        if(init==true){
            init=false;
            return State.GA;
        }
        if(endS==true){
            endS=false;        
            return State.END;
        }
        if(compass_goal == -90){
            if(compass_goal-compass>-15 && compass_goal-compass<=15) return State.GA;
            if(compass>-90 && compass<90) return State.RR;
            else return State.RL;
        }
        else if(compass_goal == 180){
            if(Math.abs(Math.abs(compass)-Math.abs(compass_goal))<=15) return State.GA;
            if(compass<0) return State.RR;
            else return State.RL;
        }
        else if(compass_goal == 90){
            if(compass_goal-compass>-15 && compass_goal-compass<=15) return State.GA;
            if(compass<90 && compass>-90) return State.RL;
            else return State.RR;

        }
        else if(compass_goal == 0){
            if(compass_goal-compass>-15 && compass_goal-compass<=15) return State.GA;
            if(compass<0) return State.RL;
            else return State.RR; 
        }
        else return State.GA;
    }
    
    public boolean targetReached(){ //chegou ao objetivo?
        if (Math.abs(x-next.getX())<=0.15 && Math.abs(y-next.getY())<=0.15){
            return true;
        }else{
            return false;
        }
    }

    public void goLeft(){
        double deltaC = Math.abs(Math.abs(compass_goal)-Math.abs(compass));
        double rot = 0.5 * deltaC;
        double l = -rot;
        double r = rot;

        cif.DriveMotors(l, r);

    }

    public void goRight(){
        double deltaC = Math.abs(Math.abs(compass_goal)-Math.abs(compass));
        double rot = 0.5 * deltaC;
        double l = rot;
        double r = -rot;

        cif.DriveMotors(l, r);
    }

    public void goInv(){
        cif.DriveMotors(0.15,-0.15);
    }

    public void goAhead(){ //yr -> y inicial -> funcao de andar para a frente
        double deltaY = next.getY() - y; //Erro do Y
        double deltaX = next.getX() - x; //Erro do X
        double l, r;
        double k = 0.75;
       
        if(Eixo()){ // ele esta virado na horizontal
            l =0.1 - k * deltaY * nivel(); //nivel corresponde a ser + ou - no eixo
            r =0.1 - k * -deltaY * nivel(); // Valor maximo de velocidade menos 
        }
        else{ //robo no eixo veritcal
            l =0.1 - k * -deltaX * nivel(); //nivel corresponde a ser + ou - no eixo
            r =0.1 - k * deltaX * nivel(); //
        }
        cif.DriveMotors(l, r);
    }
   


    //para nao ir para cima quando esta a rodado para baixo ou para a esquerda (-90 e 180)
    public double nivel(){ //alinhador dos goAheads positivo(1) ou negativo(-1)
        if(compass>-45 && compass<135) return 1;
        return -1;
    }
    public boolean Eixo(){ //true se for horizontal
        if((compass>-45 && compass<=45) || (compass>= 135) || (compass < -135)) return true;
        else return false;
    }

    // criar o caminho todo
    public void setCaminho(){
        vetor vatual=new vetor();
        vatual.setXY(x, y);
        Node target= new Node(visitaveis.getLast());
        Node head= new Node(coordsAntigas.getLast());
        Node res = aStar(head, target);
        caminho=printPath(res);     
        for(int i =0 ;i<caminho.size();i++){ //nao tira este print
            System.out.println("caminho: x-> " + caminho.get(i).getX() + " y-> " + caminho.get(i).getY());
        }
        runCaminho();
    }

    //da nos a proxima coordenada do caminho
    public void runCaminho(){ 
        next.setXY(caminho.getFirst());
        arrendAngulo2();
        if(next.getX()==coord2Frente().getX() && next.getY()==coord2Frente().getY()) compass_goal=compass;
        if(next.getX()==coord2Dir().getX() && next.getY()==coord2Dir().getY()) compass_goal=compass-90;
        if(next.getX()==coord2Esq().getX() && next.getY()==coord2Esq().getY()) compass_goal=compass+90;
        if(next.getX()==coord2Tras().getX() && next.getY()==coord2Tras().getY()) compass_goal=compass-180;
        if(compass_goal==270) compass_goal=-90;
        if(compass_goal==-270) compass_goal=90;
        arrendAngulo();        
        caminho.removeFirst();
        System.out.println("Mapping decode next x= "+next.getX()+" y= "+next.getY()+" compass_goal= "+compass_goal + " compass= " + compass);          
    }

    public void mappingDecode() throws IOException{
        Set<vetor> localViz = new HashSet<vetor>();
        //adicionar coordenada atual a lista de coordendas visitadas
        vetor abc = new vetor();
        abc.setXY(next);
        visitaveis.remove(abc);
        for(int k=0; k<vizinhos().size();k++){
            abc.addFilho(vizinhos().get(k));
        }
        coordsAntigas.add(abc);
        //MAPEAR
        //detetar paredes, se for ou | ou -, se n for entao "x"
        //se coordenada nao for parede e nao tiver sido visitada e nao estiver nos visitaveis, adiciona aos visitaveis
        vetor pf = new vetor();
        vetor pe = new vetor();
        vetor pd = new vetor();
        vetor pt = new vetor();

// adicionar qualquer coisa para ver se a posicao atual é beacon e acrescenta lo aos objetivos
        if(ground>=0){ 
            System.out.println("alvo: "+ground);
            vetor aux = new vetor(x,y);
            objetivos[ground] = aux;            
        }

        if(ParedeDireita()){
            if(!par(coordDir()))
                addToMap(coordDir(),"|");
            else
                addToMap(coordDir(),"-");
        }else{
           /* if(ground>=0){
                //vetor vaux = new vetor(objetivos[ground].getX(),objetivos[ground].getY());
                //int aux = Fcont(vaux);
                System.out.println("ENTREI" + Integer.toString(ground));
                
                addToMap(coord2Dir(), "A");
            }else{
                System.out.println("COISO");*/
                addToMap(coord2Dir(), "X");
                
          //  }
           addToMap(coordDir(), "X");
            pd.setXY(coord2Dir());
            if(!AntContem(pd)){
                localViz.add(pd);
                if (!visitaveis.contains(pd))visitaveis.add(pd);
            }
        }
        if(ParedeEsquerda()){
            if(!par(coordEsq()))
                addToMap(coordEsq(),"|");
            else 
                addToMap(coordEsq(),"-");
        }else{
            /*if(ground>=0){
               // vetor vaux = new vetor(objetivos[ground].getX(),objetivos[ground].getY());
                //int aux = Fcont(vaux);
                System.out.println("ENTREI" + Integer.toString(ground));
                
                addToMap(coord2Esq(), "A");
                //addToMap(coordEsq(), Integer.toString(ground));
            }else{
                System.out.println("COISO");*/
                addToMap(coord2Esq(), "X");
                 
           // }
           addToMap(coordEsq(), "X");
            pe.setXY(coord2Esq());
            if(!AntContem(pe)) {
                localViz.add(pe);
                if (!visitaveis.contains(pd))visitaveis.add(pe);
            }
        }
        if(ParedeFrente()){
            if(!par(coordFrente()))
                addToMap(coordFrente(),"|");
            else
                addToMap(coordFrente(),"-");
        }else{
          /*  if(ground>=0){
                //vetor vaux = new vetor(objetivos[ground].getX(),objetivos[ground].getY());
                //int aux = Fcont(vaux);
                System.out.println("ENTREI" + Integer.toString(ground));
                addToMap(coord2Frente(), "A");
              //  addToMap(coordFrente(), Integer.toString(ground));
            }else{
                System.out.println("COISO");*/
                addToMap(coord2Frente(), "X");
               
           // }
             addToMap(coordFrente(), "X");
            pf.setXY(coord2Frente());
            if(!AntContem(pf)){
                localViz.add(pf);
                if (!visitaveis.contains(pd))visitaveis.add(pf);
            }
        }
        if(ParedeTras()){
            if(!par(coordTras()))
                addToMap(coordTras(),"|");
            else
                addToMap(coordTras(),"-");
        }else{
           /* if(ground>=0){
               // vetor vaux = new vetor(objetivos[ground].getX(),objetivos[ground].getY());
               // int aux = Fcont(vaux);
               System.out.println("ENTREI" + Integer.toString(ground));
                
                addToMap(coord2Tras(), "A");
               // addToMap(coordTras(), Integer.toString(ground));
            }else{
                System.out.println("COISO");*/
                addToMap(coord2Tras(), "X");
               
           // }
             addToMap(coordTras(), "X");
            pt.setXY(coord2Tras());
            if(!AntContem(pt)){
                if (!visitaveis.contains(pd))visitaveis.add(pt);
            }
        }
        // quando visitar tudo ou se o clock estiver quase acabar
        if(visitaveis.isEmpty() || cif.GetTime()> 4990){endS=true;} 

//--------------------- calcular a posicao seguinte--------------------------------
        else{        
            if(localViz.size()==0){ 
                setCaminho();
            }
            else{
                next.setXY(localViz.iterator().next());
                arrendAngulo2();
                if(next.getX()==coord2Frente().getX() && next.getY()==coord2Frente().getY()) compass_goal=compass;
                if(next.getX()==coord2Dir().getX() && next.getY()==coord2Dir().getY()) compass_goal=compass-90;
                if(next.getX()==coord2Esq().getX() && next.getY()==coord2Esq().getY()) compass_goal=compass+90;
                if(next.getX()==coord2Tras().getX() && next.getY()==coord2Tras().getY()) compass_goal=compass-180;
            }
            if(compass_goal==270) compass_goal=-90;
            if(compass_goal==-270) compass_goal=90;
            arrendAngulo();
        }

        //System.out.println("Compass_goal antes do ArrendAngulo: "+ compass_goal);
        System.out.println("Mapping decode next x= "+next.getX()+" y= "+next.getY()+" compass_goal= "+compass_goal + " compass= " + compass);          
    }


    public LinkedList<vetor> vizinhos(){
        LinkedList <vetor> viz = new LinkedList<>();
        if(!ParedeFrente()){ viz.add(coord2Frente()); }
        if(!ParedeDireita()){ viz.add(coord2Dir()); }
        if(!ParedeEsquerda()){ viz.add(coord2Esq()); }
        if(!ParedeTras()){ viz.add(coord2Tras()); }
        return viz;
    }

//------------------------------------- MAPA MAPA MAPA -------------------------------------
    public void fillMap(){ //chamada no estado init para encher o ARRAY do mapa com " "
        for(int i=0; i<28;i++){ //percorremos as linhas
            for (int j=0; j<56; j++){ //percorremos as colunas para cada linha
                coords[i][j]=" ";
                if(i==14 && j==28) {
                    coords[i][j]= "I";                    
                }
            }
        }
    }

    public void addToMap(vetor v, String a) throws IOException { //escrever no coords a String certa 
        int col = (int) (v.getX())+28;
        int lin = 14 - (int)(v.getY());
        if(!onMap(v))coords[lin][col]= a;
    }

    public void writeMap() throws IOException{ //Escreve o mapa no file
        File fileMap = new File("mapa.txt");
        fileMap.createNewFile();
        Scanner fin = new Scanner (fileMap);
        FileWriter writeFile = new FileWriter(fileMap);
        String a = new String();
        for(int lin=1; lin<28;lin++){
            a="";
            for (int col=1; col<56; col++){
                a = a + coords[lin][col]; // a = linha toda
                if (col==55) a = a + "\n";
            }
            writeFile.write(a);
        }
        fin.close();
        writeFile.close();    
    }

    public int Fcont(vetor tmp){ //retorna onde esta o target
        for(int i = 0;i<objetivos.length;i++){
            vetor obj = new vetor();
            obj.setXY(objetivos[i].getX(),objetivos[i].getY());
            if(obj.getX()==tmp.getX() && obj.getY()==tmp.getY()){
                return i;
            }
        }
        return -1;
    }

    public void writeCaminho() throws IOException{ //Escreve o caminho no file      
        LinkedList<vetor> tmp = new LinkedList<vetor>();
        tmp = addCaminho(); 
        File fileMap = new File("caminho.txt");
        fileMap.createNewFile();
        Scanner fin = new Scanner (fileMap);
        FileWriter writeFile = new FileWriter(fileMap);
        String a = new String();
        
        for(int i=0; i<tmp.size();i++){
            int aux = Fcont(tmp.get(i)); 
            if(aux!=-1){
                a = a + tmp.get(i).getX() + " " + tmp.get(i).getY() + " #" + aux + "\n";
            }else{
                a = a + tmp.get(i).getX() + " " + tmp.get(i).getY() + "\n";
            }
        }  
        
        writeFile.write(a);
        fin.close();
        writeFile.close();  
    }
    
    public LinkedList<vetor> setCaminhoFinal(vetor posicaoVer, vetor posicaoAtual){
        LinkedList<vetor> tmp = new LinkedList<vetor>();
        Node target= new Node(posicaoVer);
        Node head= new Node(posicaoAtual);
        Node res = aStar(head, target);
        tmp=printPath(res);     
        
        return tmp;
    }

    public LinkedList<vetor> addCaminho(){ //faz caminho de todos os beacons detetados
        LinkedList<vetor> tmp2 = new LinkedList<vetor>();
        LinkedList<vetor> tmp = new LinkedList<vetor>();

        if(0!=objetivos[0].getX() || 0 != objetivos[0].getY()){ //se a posicao inicial nao for um target
            vetor AuxposicaoVer = new vetor();
            AuxposicaoVer.setXY(objetivos[0].getX(),objetivos[0].getY());
            vetor AuxposicaoAtual = new vetor();
            AuxposicaoAtual.setXY(0,0);
            tmp2 = setCaminhoFinal(AuxposicaoVer, AuxposicaoAtual);
            for(int j =0 ;j<tmp2.size();j++){ 
                tmp.addLast(tmp2.get(j));                
            }
        }   
  
        for(int i=0; i<objetivos.length-1;i++){  
            vetor posicaoVer = new vetor();
            posicaoVer.setXY(objetivos[i+1].getX(),objetivos[i+1].getY());
            vetor posicaoAtual = new vetor();
            posicaoAtual.setXY(objetivos[i].getX(),objetivos[i].getY());
            tmp2 = setCaminhoFinal(posicaoVer, posicaoAtual);
            for(int j =0 ;j<tmp2.size()-1;j++){
                if(tmp2.get(j).getX() != tmp2.get(j+1).getX() || tmp2.get(j).getY() != tmp2.get(j+1).getY()){
                   tmp.add(tmp2.get(j));
                }
            }
        }
        vetor AuxposicaoVer = new vetor();
        AuxposicaoVer.setXY(0,0);
        vetor AuxposicaoAtual = new vetor();
        AuxposicaoAtual.setXY(objetivos[objetivos.length-1].getX(),objetivos[objetivos.length-1].getY());
        tmp2 = setCaminhoFinal(AuxposicaoVer, AuxposicaoAtual);
        for(int j =0 ;j<tmp2.size();j++){ 
            tmp.addLast(tmp2.get(j));                
        } 

       /* for(int j =0 ;j<tmp.size();j++){              
            System.out.println("caminho Final: x-> " + tmp.get(j).getX() + " y-> " + tmp.get(j).getY());
        }*/
       
        return tmp;
    }

    public boolean SeeBeacons(){ //para ver se ja passou em todos os beacons
        int count=0;
        for (int i = 0;i<objetivos.length;i++){
            if (objetivos[i]!= null){
                count++;
            }
        }
        if(count == nbeacons){
            return true;
        }
        return false;
    }

    public boolean onMap(vetor v) {  //Verificar se as coordenadas dadas já estão preenchidas
        int col = (int) (v.getX())+28;
        int lin = 14 - (int) (v.getY());
        if(coords[lin][col].equals("|") || coords[lin][col].equals("X") || coords[lin][col].equals("-") || coords[lin][col].equals("I")){
            return true;
        } 
        return false;
    }

    public boolean par(vetor v){//impar horizontal, par vertical
        if(v.getX()%2==0) return true;
        return false;
    }


    static void print_usage() {
        System.out.println("Usage: java jClientC4 [--robname <robname>] [--pos <pos>] [--host <hostname>[:<port>]] [--map <map_filename>]");
    }

    public void printMap() {
           if (map==null) return;

           for (int r=map.labMap.length-1; r>=0 ; r--) {
               System.out.println(map.labMap[r]);
           }
    }

    public static LinkedList<vetor> printPath(Node target){    
        if(target==null){
            return null;
        } 
        LinkedList<vetor> path = new LinkedList<>();
        while(target.parent != null){
            path.add(target.value);
            target = target.parent;
        }
        path.add(target.value);
        Collections.reverse(path);
        return path;
    }


    //AUXILIARES
    //ver se existe parede nas 4 direcoes
    public boolean ParedeFrente(){
        if(irSensor0>=1.3) return true;
        return false;
    }
    public boolean ParedeTras(){
        if(irSensor3>=1.3) return true;
        return false;
    }
    public boolean ParedeDireita(){
        if(irSensor2>=1.3) return true;
        return false;
    }
    public boolean ParedeEsquerda(){
        if(irSensor1>=1.3) return true;
        return false;
    }

    //Dar coordenadas
    public vetor coordEsq(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x,y+1);
        else if(compass>=45 && compass <135) v.setXY(x-1,y);
        else if(compass==-90) v.setXY(x+1,y);
        else v.setXY(x,y-1);
        return v;
    }
    public vetor coordDir(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x,y-1);
        else if(compass>=45 && compass <135) v.setXY(x+1,y);
        else if(compass<-45 && compass >= -135) v.setXY(x-1,y);
        else v.setXY(x,y+1);
        return v;
    }
    public vetor coordFrente(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x+1,y);
        else if(compass>=45 && compass <135) v.setXY(x,y+1);
        else if(compass<-45 && compass >= -135) v.setXY(x,y-1);
        else v.setXY(x-1,y);
        return v;
    }
    public vetor coordTras(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45)  v.setXY(x-1,y);
        else if(compass>=45 && compass <135)  v.setXY(x,y-1);
        else if(compass<-45 && compass >= -135)  v.setXY(x,y+1);
        else  v.setXY(x+1,y);
        return v;
    }

    //2 coordenadas -> centro da celula
    public vetor coord2Esq(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x,y+2);
        else if(compass>=45 && compass <135) v.setXY(x-2,y);
        else if(compass<-45 && compass >= -135) v.setXY(x+2,y);
        else v.setXY(x,y-2);
        return v;
    }
    public vetor coord2Dir(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x,y-2);
        else if(compass>=45 && compass <135) v.setXY(x+2,y);
        else if(compass<-45 && compass >= -135) v.setXY(x-2,y);
        else v.setXY(x,y+2);
        return v;
    }
    public vetor coord2Frente(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x+2,y);
        else if(compass>=45 && compass <135) v.setXY(x,y+2);
        else if(compass<-45 && compass >= -135) v.setXY(x,y-2);
        else v.setXY(x-2,y);
        return v;
    }
    public vetor coord2Tras(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45)  v.setXY(x-2,y);
        else if(compass>=45 && compass <135)  v.setXY(x,y-2);
        else if(compass<-45 && compass >= -135)  v.setXY(x,y+2);
        else  v.setXY(x+2,y);
        return v;
    }

    public void arrendAngulo(){
        if(compass_goal<45 && compass_goal >=-45)  compass_goal=0;
        else if(compass_goal>=45 && compass_goal <135)  compass_goal=90;
        else if(compass_goal<-45 && compass_goal >= -135)  compass_goal=-90;
        else compass_goal=180;
    }

    public void arrendAngulo2(){
        if(compass<45 && compass >=-45)  compass=0;
        else if(compass>=45 && compass <135)  compass=90;
        else if(compass<-45 && compass >= -135)  compass=-90;
        else compass=180;
    }
    public boolean AntContem(vetor v){
        for(int i=0; i<coordsAntigas.size(); i++){
            if(coordsAntigas.get(i).equals(v)) return true;
        }
        return false;
    }

    //VARIAVEIS

    private String robName;
    private double irSensor0, irSensor1, irSensor2, irSensor3, compass, compass_goal, x, y, x0,y0;
    //sensores, bussola, bussola para o objetivo, coordenadas, coordenadas iniciais
    // compass_goal para onde ele vai ter de rodar
    private beaconMeasure beacon;
    private int ground;
    private boolean collision,init, endS; //initial position?
    private vetor next = new vetor(); //para onde vai a seguir
    private State state;
    private int beaconToFollow;
    private String[][] coords = new String[28][56]; //linhas (Y), colunas (X), mapa completo
    private LinkedList<vetor> coordsAntigas = new LinkedList<vetor>(); //linhas, colunas
    private LinkedList<vetor> visitaveis = new LinkedList<vetor>();
    private LinkedList<vetor> caminho = new LinkedList<vetor>();
    private int nbeacons;
    private vetor[] objetivos; //variavel com as coordenadas dos goals

public class Node implements Comparable<Node> {
        // Id for readability of result purposes
        private int idCounter = 0;
        public int id;
        public vetor value;
  
        // Parent in the path
        public Node parent = null;
  
        public List<Edge> neighbours;
        

        public double g = 1; 
        public double h; 
        // Evaluation functions
        public double f = g+h;
       
  
        public Node(vetor v){
              value = v;
              h=0;
              this.id = idCounter++;
              this.neighbours = new ArrayList<>(); 
        }
  
        @Override
        public int compareTo(Node n) {
              return Double.compare(this.f, n.f);
        }
  
        public class Edge {
              Edge(int weight, Node node){
                    this.weight = 1;
                    this.node = node;
              }
  
              public int weight;
              public Node node;
        }
        public void addBranch(int weight, Node node){
            Edge newEdge = new Edge(weight, node);
            neighbours.add(newEdge);
        }

        public double calculateHeuristic(Node target){
          h=Math.abs(value.getX()-target.value.getX())+Math.abs(value.getY()-target.value.getY());  //Manhattan 
          return h;
        }

        public LinkedList<Edge> actions(){
            Node[] n = new Node[4];
            LinkedList<Edge> ret = new LinkedList<Edge>();
            for(int l=0; l<coordsAntigas.size();l++){
                vetor k= new vetor(coordsAntigas.get(l).getX(), coordsAntigas.get(l).getY(), coordsAntigas.get(l).filhos);
                if(k.getX()==value.getX() && k.getY()==value.getY()){
                    value.filhos=k.filhos;
                }
            }
            for(int i=0; i<value.filhos.size(); i++){
                n[i]= new Node(value.filhos.get(i)); 
                n[i].parent=this;
                ret.add(new Edge(1,n[i]));
            }    
            return ret;
        }
    }

    public static Node aStar(Node start, Node target){

        PriorityQueue<Node> closedList = new PriorityQueue<>();
        PriorityQueue<Node> openList = new PriorityQueue<>();
        closedList.clear();
        openList.clear();
        start.f = start.g + start.calculateHeuristic(target);
        openList.add(start);     
        while(!openList.isEmpty()){
            Node n = openList.peek();
            if(n.value.equals(target.value)){
                return n;
            }
            
            n.neighbours=n.actions();
            for(Node.Edge edge : n.neighbours){
                Node m = edge.node;
                double totalWeight = n.g + edge.weight;
    
                if(!openList.contains(m) && !closedList.contains(m)){
                    m.parent = n;
                    m.g = totalWeight;
                    m.f = m.g + m.calculateHeuristic(target);
                    openList.add(m);
                } 
            }
            openList.remove(n);
            closedList.add(n);
        }
        return null;
    }
        
    
    public class vetor{
        private double x;
        private double y;
        private LinkedList<vetor> filhos = new LinkedList<vetor>();
        
        public vetor(){}
        public vetor(double x, double y){ 
            this.x=x;
            this.y=y;
        }
        public vetor(double x, double y, LinkedList<vetor> filhos){
            this.x=x;
            this.y=y;
            this.filhos=filhos;
        }
        public double getX(){
            return x;
        }
        public double getY(){
            return y;
        }
        public void addFilho(vetor v){
            filhos.add(v);
        }
        public void setX(double x){
            this.x=x;
        }
        public void setY(double y){
            this.y=y;
        }
        public void setXY(vetor v){
            this.x=v.getX();
            this.y=v.getY();
        }
        public void setXY(double xf, double yf){
            double nx = Double.valueOf(Math.round(xf));
            double ny = Double.valueOf(Math.round(yf));
            x=nx;
            y=ny;
        }
        public void printV(){
            System.out.println(x + " "+y);
            Iterator pre = filhos.iterator();        
            while(pre.hasNext()){
                vetor v = (vetor) pre.next();
                System.out.println(" filho: "+ v.x +" "+v.y);
            }
        }

        @Override
        public String toString(){
            String a = new String();
            a = x +" "+y;
            return a;
        }


        @Override
        public boolean equals(Object o){
            // If the object is compared with itself then return true 
            if (o == this) {
                return true;
            }

            /* Check if o is an instance of Complex or not
            "null instanceof [type]" also returns false */
            if (!(o instanceof vetor)) {
                return false;
            }
            
            // typecast o to Complex so that we can compare data members
            vetor v1 = (vetor) o;
            
            // Compare the data members and return accordingly
            return Double.compare(x, v1.x) == 0 && Double.compare(y, v1.y) == 0;
        }
    }

};


