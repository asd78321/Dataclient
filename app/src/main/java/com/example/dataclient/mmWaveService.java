package com.example.dataclient;

import android.annotation.SuppressLint;
import android.app.Service;
import android.content.Context;
import android.content.Intent;

import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.PowerManager;
import android.util.Log;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.InputStream;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.Calendar;
import java.lang.Math; //abs()

import android.content.res.AssetFileDescriptor;

import org.tensorflow.lite.Interpreter;

import java.net.Socket;//Socket()
import java.net.InetAddress;


public class mmWaveService extends Service {
    private static mmWaveService mmWaveInstance;
    private static Context mContext;

    public static mmWaveService getInstance() {
        if (mmWaveInstance == null) {
            synchronized (mmWaveService.class) {
                if (mmWaveInstance == null) {
                    mmWaveInstance = new mmWaveService();
                }
            }
        }
        return mmWaveInstance;
    }

    private static final String ClassName = mmWaveService.class.getSimpleName();

    private static final int STACK_SIZE = 30;
    private static final int PACKAGE_HEADER_LENGTH = 52;
    private static final int VITAL_SIGNS_CONTENT_LENGTH = 128;
    private static final int NIBBLE = 4;
    private static final int TLV_HEADER_LENGTH = NIBBLE * 2;
    private static final int TLV_TARGET_LIST_LENGTH = NIBBLE * 10;
    private static final int TLV_POINT_CLOUD_LENGTH = NIBBLE * 5;
    private static final int BREATH_FRAME_THRE = 20;
    private static final int FALL_DETECTION_THRE = 9;
    private static final int IN_AREA_THRE = 10; //threshold for count_ManIn/count_NotManIn
    private static final float RCSSTDRAW_AVG_THRE = 700.0f;
    private static final float FIRST_RCSSTDRAW_THRE = 400.0f;

    private static String macAddress = "00:00:00:00:00:00";
    private static String empty = "empty";

    private List<Byte> dataList = new ArrayList<>();
    private int headerCount = 0;

    private int TLV_LieEnabled;
    private int TLV_framenum;
    private int numTLVItem;
    private float FwVersion;
    private String sFwVersion;
    private boolean isRecord = true;
    private float TLV_PosX;
    private float TLV_PosZ;

    private int number;
    private int numbertemp = -1;
    private int count = 0;
    private byte[] msg;
    private byte[] nowFramePointCloud;

    private ReadPktThread pktThread;
    private ReadLogThread logThread;

    private Thread socketClientThread;
    private Socket clientSocket;
    private String tmp;
    private boolean isParsing = false, Sendsignal = true;


    int numLabel = 0;
    int numLabel_temp = -1;
    String[] humanstates = {"stand", "sit", "fall", "getup"};

    int humanstate = 0;
    int progress = 0;
    boolean fallsignal = false;

    long startTime = 0;
    long endTime = 0;
    long startTime_bed = 0;
    long endTime_bed = 0;
    long startTime_warning = 0;
    long endTime_warning = 0;
    long startTime_fall = 0;
    long endTime_fall = 0;
    long warning = 0;
    //int  leavebed_count = 0; 

    private DetectRadarThread detRadarThread;
    public volatile boolean isRadarPending = false;
    public Object lock = new Object();


    final int SIZE_SAMPLE = 512;
    float[][][] stack_pixel = new float[2][9][50 * 30];
    final int ENERGY_BUF_SIZE = 50;
    final int BR_OUT_BUF_SIZE = 10;
    private Interpreter tflite;

    //LED controller
    private static final String TAG = "LedSample";
    private static final int LED_OFF = 0;
    private static final int LED_FLASH_ON = 1;
    private static final int LED_FLASH_OFF = 2;
    private static final int LED_ON = 3;
    public final static int LED_DELAY_TIME = 300;
    private static int ledAction = 2;

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    @Override
    public void onCreate() {

    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.d(ClassName, "Start mmWaveService");

        mContext = mmWaveService.this;


//        try {
//            socketClientThread = new Thread(Client);
//            socketClientThread.start();
//            Log.d(ClassName, "Start SocketThread!!");
//        } catch (Exception e) {
//            Log.d(ClassName, "ConnectService:" + e.getMessage());
//        }


        if (logThread == null) {
            logThread = new ReadLogThread(mContext);
            logThread.start();
            Log.d(ClassName, "Start logThread!!");
        }

        if (pktThread == null) {
            pktThread = new ReadPktThread();
            pktThread.start();
            Log.d(ClassName, "Start pktThread!!!");
        }


        return START_STICKY;
    }

    private Runnable Client = new Runnable() {
        @Override
        public void run() {
            InetAddress serverIp;
            while (true) {
                try {
                    serverIp = InetAddress.getByName("10.1.1.42"); // 10.1.1.38 // 192.168.0.105
                    int serverPort = 5050;
                    clientSocket = new Socket(serverIp, serverPort);
                    break;
                } catch (IOException e) {
                    Log.d(ClassName, "SocketService:" + e.getMessage());
                }
            }
            Log.d(ClassName, "create Interpreter");
            try {
                tflite = new Interpreter(loadModelFile("model"));
            } catch (IOException e) {
                e.printStackTrace();
            }
            if (clientSocket.isConnected()) {
                Log.d(ClassName, "conencted server!!!");
                while (clientSocket.isConnected()) {

                    if (true) {
                        try {
                            DataOutputStream bw = new DataOutputStream(new BufferedOutputStream(clientSocket.getOutputStream()));
//                            if (numbertemp != number && msg != null) {
//                                bw.write(msg);
//                                bw.flush();
//                                Log.d(ClassName, "send_Frame:" + number);
//                            }
//                            numbertemp = number;
                            if (Sendsignal != true) {
                                bw.writeUTF(tmp);
                                bw.flush();
                                Sendsignal = true;
                            }
                        } catch (Exception e) {
                            Log.d(ClassName, "Socketsend:" + e.getMessage());
                            continue;
                        }
                    }
                }
            }
        }
    };

    private class DetectRadarThread extends Thread {
        Context ctx;

        public DetectRadarThread(Context ctx) {
            this.ctx = ctx;
        }

        @Override
        public void run() {
            long startTime = 0;
            long diffTime = 0;
            long currTime = 0;
            long pendingStartTime = 0;
            //Need time to send config to trigger radar, wait for 10s to detect radar.
            try {
                Thread.sleep(10000);
                while (true) {
                    synchronized (lock) {
                        if (isRadarPending) {
                            currTime = System.currentTimeMillis();
                            if (startTime == 0) {
                                startTime = currTime;
                                pendingStartTime = startTime;
                            } else {
                                diffTime = currTime - startTime;
                            }
                            if (diffTime > 3000) { // 3s
                                Log.d(ClassName, "call sendHealthReport(), radar is no response over than 3s.");

                                startTime = currTime;
                            }
                            if ((currTime - pendingStartTime) > 10000) { // 10s
                                Log.d(ClassName, "endTime>10000, try to reboot!!!!!!!");
                                PowerManager pm = (PowerManager) ctx.getSystemService(Context.POWER_SERVICE);
                                pm.reboot(null);
                            }
                        } else {
                            startTime = 0;
                            endTime = 0;
                            currTime = 0;
                            pendingStartTime = 0;
                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private class ReadPktThread extends Thread {
        @Override
        public void run() {
            Byte[] delimiter = {(byte) 0x02, (byte) 0x01, (byte) 0x04, (byte) 0x03, (byte) 0x06, (byte) 0x05, (byte) 0x08, (byte) 0x07};
            byte[] byteData;
            int numRead = -1;

            String line;
            String uart_test = "/system/bin/uart_test";

            try {
                tflite = new Interpreter(loadModelFile("model"));
            } catch (IOException e) {
                e.printStackTrace();
            }



            try {
                Process p = Runtime.getRuntime().exec(uart_test);
                Log.d(ClassName, "execute uart_test(pkt)");


                BufferedReader br = new
                        BufferedReader(new InputStreamReader(p.getInputStream()));
                while (true) {
                    line = br.readLine();
                    if (line == null)
                        continue;

                    byteData = stdOutputToByte(line);
                    numRead = byteData.length;

                    if (numRead == -1) {
                        Log.d(ClassName, " numRead is -1, abnormal !!!!!! ");

                        continue;

                    }

                    byte[] tempData = new byte[numRead];
                    System.arraycopy(byteData, 0, tempData, 0, numRead);

//                    Log.d(ClassName, "Data length:"+tempData.length);

                    for (byte b : tempData) {
                        dataList.add(b);

                        if (delimiter[headerCount] == b) {
                            headerCount++;
                        } else {
                            headerCount = 0;
                        }

                        if (headerCount == delimiter.length) {
                            dataList = dataList.subList(0, dataList.size() - delimiter.length);
                            Log.d(ClassName, " call parser!");
                            parseTLV(listByteToByteArray(dataList));
                            isParsing = true;
                            dataList = new ArrayList<Byte>(Arrays.asList(delimiter));
                            headerCount = 0;
                        }
                    }
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }


    public class ReadLogThread extends Thread {

        Context ctx;

        private String mfilename;
        private DataOutputStream dos;
        private InputStream in;
        private InputStreamReader input;
        private Scanner sc;
        private SimpleDateFormat mSimpleDateFormat = new SimpleDateFormat("YYYY-MM-dd HH:mm:ss.SSS");
        private int filecnt = 0;
        private long fileSizeInBytes;
        private long fileSizeInKB;
        private long fileSizeInMB;
        SimpleDateFormat date = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss,SSS");
        Date currentTime = new Date();
        String[] ReadLogThread = new String[STACK_SIZE];
        String[] ReadLogThread_dbgMsg;
        int num = 0;


        public ReadLogThread(Context ctx) {
            this.ctx = ctx;

        }


        protected void finalize() {
            this.sc.close();
        }

        public String convert(int t) {
            String s = Integer.toString(t);
            if (t < 10) {
                s = "0" + s;
            }
            return s;
        }

        public String getTimeString() {
            Calendar cal = Calendar.getInstance();
            int sec = cal.get(Calendar.SECOND);
            int min = cal.get(Calendar.MINUTE);
            int hour = cal.get(Calendar.HOUR_OF_DAY);
            int date = cal.get(Calendar.DATE);
            int month = cal.get(Calendar.MONTH) + 1;
            int year = cal.get(Calendar.YEAR);
            return convert(year) + convert(month) + convert(date) + convert(hour) + convert(min) + convert(sec);
        }

        @Override
        public void run() {

            String line = null;
            String uart_log = "/system/bin/uart_log";
            int reboot_count = 0;
            long startTime = 0;
            long endTime = 0;

            long startLogTime = 0;
            long currLogTime = 0;

            SimpleDateFormat mAllFormat = new SimpleDateFormat("yyyyMMddHHmm");
            long curtime = System.currentTimeMillis();
            String now_time = mAllFormat.format(new Date(curtime));
            String oldmac = macAddress;

            String newmac = oldmac.replaceAll(":", "_");

            try {

                Process p = Runtime.getRuntime().exec(uart_log);
                Log.d(ClassName, "execute uart_log(config/log)");


                BufferedReader br = new
                        BufferedReader(new InputStreamReader(p.getInputStream()));


                long now, last, diff;

                last = System.currentTimeMillis();


                while (true) {
                    Date d = new Date(System.currentTimeMillis());
                    String logtime = mSimpleDateFormat.format(d);
//                    Log.d(ClassName, "Time: "+logtime);
                    synchronized (lock) {
                        isRadarPending = true;
                    }
                    line = br.readLine();
                    synchronized (lock) {
                        isRadarPending = false;
                    }


                    if ((line != null) && (!line.equals(""))) {

                        now = System.currentTimeMillis();
                        diff = now - last;

                        if (startLogTime == 0) {
                            startLogTime = System.currentTimeMillis();
                            currLogTime = startLogTime;
                        } else {
                            currLogTime = System.currentTimeMillis();
                            if ((currLogTime - startLogTime) >= 3000) { //3 seconds
                                Log.d(ClassName, "call sendHealthReport(), radar is working.");
                                startLogTime = currLogTime;
                            }
                        }

                        last = now;
                    }
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }

    }

    float range = 0;

    public float getArrayAvg(float[] fArray, int size) {
        int i;
        float total = 0;

        for (i = 0; i < size; i++) {
            total += fArray[i];
        }
        return (total / size);
    }


    private void parseTLV(byte[] fileBytes) {
        SimpleDateFormat date = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss,SSS");
        String filePath = "/data/data/state/logs/test.txt";
        String Filemessage = "狀態";
        Date currentTime = new Date();
        String[] tragetObjectData = new String[5];
        String[] pointCloudData = new String[10];
        String[] parseTLV = new String[STACK_SIZE];
        String[] parseTLV_dbgMsg;
        String comm = "";
        int num = 0;
        //parseTLV1 = new String[num];

        final int OFFSET_RANGEPROFILE_BEGIN = 196;
        final Double ALPHA_RCS = 0.2;
        float cm_Breath;
        float PointX, PointY, PointZ;

        double breath_FFT;
        double breath_Peak;

        int numTLV = 0;
        int TLVType[] = {0, 0, 0, 0, 0, 0};
        int TLVLength[] = {0, 0, 0, 0, 0, 0};

        int IDVirtual = 0;
        //int IDReal = 0;
        int FrameLengthReal;

        boolean TLV0A_Exist = false;
        boolean prefall = false; //Add,2020/6/15


        File path = new File("/data/data/state/logs/");
        if (!path.exists()){
            path.mkdirs();
        }

        ArrayList<Integer> Target_IDList = new ArrayList<Integer>();


        // Get the real length of the package
        FrameLengthReal = fileBytes.length;

        // Return if the package only have header
        if (FrameLengthReal < PACKAGE_HEADER_LENGTH)
            return;

        int packetLength = ByteArray2Int(Arrays.copyOfRange(fileBytes, NIBBLE * 5, NIBBLE * 6));
        int frameNumber = ByteArray2Int(Arrays.copyOfRange(fileBytes, NIBBLE * 6, NIBBLE * 7));

        number = frameNumber;
        Log.d(ClassName, "Framenumber: " + number);

        if (FrameLengthReal != packetLength)
            return;

//        msg = fileBytes;
        numTLV = (byte) fileBytes[49] << 8 | (byte) fileBytes[48];


        // This code iterates the TLV package
        try {
            for (int i = 0; i < numTLV; i++) {
                TLVType[i] = geTLVType(fileBytes, PACKAGE_HEADER_LENGTH + sumArray(TLVLength));
                // TLV object length, including TLV header
                TLVLength[i] = geTLVLength(fileBytes, PACKAGE_HEADER_LENGTH + sumArray(TLVLength));
                //if (TLVType[i] == 0x09 || TLVType[i] == 0x02)
                //TLVLength[i] += TLV_HEADER_LENGTH;
            }
        } catch (ArrayIndexOutOfBoundsException e) {
            Log.d(ClassName, "Package Exception: " + e);
            return;
        }

///////////////////////////////////Parser////////////////////////////
        for (int i = 0; i < numTLV; i++) {
            int prefixLength = PACKAGE_HEADER_LENGTH + TLV_HEADER_LENGTH + sumArrayRange(TLVLength, 0, i - 1);
            byte[] byteOutput = Arrays.copyOfRange(fileBytes, prefixLength, prefixLength + TLVLength[i]);

            if (TLVType[i] == 0x06 && TLVLength[i] >= 28) {
                int TLVPoints = (TLVLength[i] - TLV_HEADER_LENGTH) / TLV_POINT_CLOUD_LENGTH;
                byte[] bytesPointCloud = Arrays.copyOfRange(byteOutput,
                        0, TLVPoints * TLV_POINT_CLOUD_LENGTH);
                nowFramePointCloud = bytesPointCloud;
                msg = nowFramePointCloud;
/////////////////////////////////Interpreter////////////////////////////////////
                final float[][] pointcloud = BytesPoint2float(msg);
//                Log.d(ClassName,"call pointcloud!");
                if (pointcloud != null) {
                    float[][] pixel = voxalize(pointcloud[0], pointcloud[1], pointcloud[2], 50, 30, 50);
                    stack_pixel = stackSlid_pixel(pixel, stack_pixel, count);
//                    Log.d(ClassName,"call stack!");
                    count += 1;
                    if (count > 8 && count % 9 ==1 ) {
//
                        float[] input1 = flatteninput(stack_pixel[0]);
                        float[] input2 = flatteninput(stack_pixel[1]);

                        ByteBuffer byteinput1 = floatBuffer2byteBuffer(input1, input1.length);
                        ByteBuffer byteinput2 = floatBuffer2byteBuffer(input2, input2.length);
                        Object[] inputs = {byteinput1, byteinput2};
//                        Log.d(ClassName,"call input!");
                        Map<Integer, Object> outputs = new HashMap<>();
                        final float[][] output_0 = new float[1][7];
                        outputs.put(0, output_0);
//
                        tflite.runForMultipleInputsOutputs(inputs, outputs);


                        numLabel = getOutputLabelindex(output_0);
                        String nowLog = "";
                        if (numLabel != numLabel_temp) {
//                            Log.d(ClassName, "now:" + numLabel + " past:" + numLabel_temp);
                            switch (numLabel) {
                                case 0:
                                case 2:
                                case 6:
                                    humanstate = 0;
//                                    Log.d(ClassName, numLabel + " Turn on!");
                                    ledHandler.removeMessages(LED_FLASH_ON);
                                    ledHandler.removeMessages(LED_FLASH_OFF);
                                    ledHandler.removeMessages(LED_OFF);

                                    ledHandler.sendEmptyMessage(LED_ON);
                                    Log.d(ClassName, "flash end!");
                                    fallsignal = false;
                                    break;

                                case 3:
                                case 1:
                                case 4:
                                    humanstate = 1;

                                    ledHandler.removeMessages(LED_FLASH_ON);
                                    ledHandler.removeMessages(LED_FLASH_OFF);
                                    ledHandler.removeMessages(LED_ON);

                                    ledHandler.sendEmptyMessage(LED_OFF);
                                    Log.d(ClassName, "flash end!");
                                    fallsignal = false;
                                    break;

                                case 5: //fall,LED flash
                                    humanstate = 2;
                                    ledHandler.removeMessages(LED_ON);
                                    ledHandler.removeMessages(LED_OFF);

                                    ledHandler.sendEmptyMessage(LED_FLASH_ON);
                                    Log.d(ClassName, "flash on!");
                                    fallsignal = true;
                                    break;
                            }
                        }
                        numLabel_temp = numLabel;
                        Date LogTime = new Date();
                        String sLogTime = date.format(LogTime);
                        nowLog = sLogTime + " " + humanstates[humanstate]+"\n";;
                        writeToFile(nowLog,filePath,true);

//                        tmp = humanstates[humanstate];
//                        tmp = FindProbIndex(output_0);
//                        Sendsignal = false;
                        Log.d(ClassName, "FrameNumber:" + String.valueOf(frameNumber) + ", PredictionResult:" + humanstates[humanstate] + ", fallsignal:" + fallsignal + ", result:" + numLabel);
                    }
                }
//////////////////////////////////////////////////////////////////////////////////////////////////
            }

//            if (TLVType[i] == 0x07 && TLVLength[i] >= 48) {
//                numTLVItem = (TLVLength[i] - TLV_HEADER_LENGTH) / TLV_TARGET_LIST_LENGTH;
//
//                Log.d(ClassName, "ParseTLV:numTLVItem: " + numTLVItem);
//
//                for (int j = 0; j < numTLVItem; j++) {
//                    int TLVTID = ByteArray2Int(Arrays.copyOfRange(byteOutput,
//                            j * TLV_TARGET_LIST_LENGTH, NIBBLE + j * TLV_TARGET_LIST_LENGTH));
//                    float TLV_PosX = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 2 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_PosY = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 2 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 3 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_PosZ = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 3 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 4 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_DimX = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 7 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 8 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_DimY = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 8 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 9 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_DimZ = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 9 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 10 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//
//
//                    if (isRecord) {
//                        tragetObjectData[0] = Integer.toString(frameNumber);
//                        tragetObjectData[1] = Float.toString(TLV_PosX);
//                        tragetObjectData[2] = Float.toString(TLV_PosY);
//                        tragetObjectData[3] = Float.toString(TLV_PosZ);
//                        tragetObjectData[4] = Integer.toString(TLVTID);
//
//
//                    }
//                }
//            }
//
//            if (TLVType[i] == 0x08 && TLVLength[i] > 8) {
//                int numTLVTargetIndex = TLVLength[i] - TLV_HEADER_LENGTH;
//                byte[] TLVTargetIndex = Arrays.copyOfRange(byteOutput, 0, TLVLength[i] - TLV_HEADER_LENGTH);
//
//                int numNowFramePointCloud = (nowFramePointCloud.length) / TLV_POINT_CLOUD_LENGTH;
//                Log.d(ClassName, "FrameNumber: " + frameNumber + "PointCloud: " + numNowFramePointCloud);
//
//                String[][] sendmsg = new String[numNowFramePointCloud][4];
//                if (numTLVTargetIndex == numNowFramePointCloud) {
//                    for (int j = 0; j < numTLVTargetIndex; j++) {
//                        int valTLVTargetIndex = TLVTargetIndex[j] & 0xFF;
//                        float Range = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                j * TLV_POINT_CLOUD_LENGTH, NIBBLE + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                        float ThetaXY = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                NIBBLE + j * TLV_POINT_CLOUD_LENGTH, NIBBLE * 2 + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                        float PhiZ = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                NIBBLE * 2 + j * TLV_POINT_CLOUD_LENGTH, NIBBLE * 3 + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                        float Doppler = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                NIBBLE * 3 + j * TLV_POINT_CLOUD_LENGTH, NIBBLE * 4 + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                        float SNRValue = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                NIBBLE * 4 + j * TLV_POINT_CLOUD_LENGTH, NIBBLE * 5 + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//
//                        PointX = (float) (Range * Math.cos(PhiZ) * Math.sin(ThetaXY));
//                        PointY = (float) (Range * Math.cos(PhiZ) * Math.cos(ThetaXY));
//                        PointZ = (float) (Range * Math.sin(PhiZ));
//
//                        if (isRecord) {
//                            pointCloudData[0] = Integer.toString(frameNumber);
//                            pointCloudData[1] = Float.toString(PointX);
//                            pointCloudData[2] = Float.toString(PointY);
//                            pointCloudData[3] = Float.toString(PointZ);
//                            pointCloudData[4] = Integer.toString(valTLVTargetIndex);
//                            pointCloudData[5] = Float.toString(Range);
//                            pointCloudData[6] = Float.toString(ThetaXY);
//                            pointCloudData[7] = Float.toString(PhiZ);
//                            pointCloudData[8] = Float.toString(Doppler);
//                            pointCloudData[9] = Float.toString(SNRValue);
//                        }
////                        sendmsg[j][0] = pointCloudData[0];
////                        sendmsg[j][1] = pointCloudData[1];
////                        sendmsg[j][2] = pointCloudData[2];
////                        sendmsg[j][3] = pointCloudData[3];
//                    }
////                    msg = sendmsg;
//                }
//            }
//
//
        }
///////////////////////////////////Parser////////////////////////////
    }
    // write Log offline
    private void writeToFile(String message, String filePath, boolean isAppendMode){
        File mFile = new File(filePath);
        try {
            boolean isFileExist = mFile.exists();
            if(!isFileExist) {
                mFile.createNewFile();
            }
            FileWriter fw = new FileWriter(mFile, isAppendMode);
            PrintWriter pw = new PrintWriter(new BufferedWriter(fw));
            try {
                Log.d(ClassName,"Wrote:" + message);
                pw.write(message);
                pw.flush();
                pw.close();
                pw = null;
                fw = null;
            } catch (Exception e) {
                Log.i("Test", "writeLog() filePath = " + filePath + " writeTitle exception");
                e.printStackTrace();
            }
        } catch (Exception e) {
            Log.d("Test", "writeLog() filePath = " + filePath + " createNewFile exception");
            e.printStackTrace();
        }
    }

    // LED controller
    @SuppressLint("HandlerLeak")
    private Handler ledHandler = new Handler() {

        @Override

        public void handleMessage(Message msg) {

            super.handleMessage(msg);

            Log.d(TAG, "ledHandler start");


            String comm = "";


            switch (msg.what) {

                case LED_FLASH_ON:

                    progress = 255;

                    ledAction = LED_FLASH_OFF;

                    break;

                case LED_FLASH_OFF:

                    progress = 0;

                    ledAction = LED_FLASH_ON;

                    break;

                case LED_OFF:

                    progress = 0;

                    ledAction = LED_OFF;

                    break;

                case LED_ON:

                    progress = 255;

                    ledAction = LED_ON;
                    break;
            }

            //EVT

            comm = "echo " + progress + " > /sys/class/leds/yellow_pwm/brightness";
            String command1[] = {"sh", "-c", comm};
            runShellCommand(command1);

            //EVT & DVT

            comm = "echo " + progress + " > /sys/class/leds/yellow/brightness";
            String command2[] = {"sh", "-c", comm};
            runShellCommand(command2);

            ledHandler.sendEmptyMessageDelayed(ledAction, LED_DELAY_TIME);


        }

    };


    public static void runShellCommand(String command[]) {

        try {

            Process p = Runtime.getRuntime().exec(command);
            p.waitFor();
            int exitVal = p.exitValue();
            Log.d(TAG, "runShellCommand: " + command + " (status: " + exitVal + ")");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private float[][] BytesPoint2float(byte[] fileBytes) {
        int Headerlength = 0;
        int PointCloudlenght = 20;
        int point_count = fileBytes.length / 20;
        float[][] point_cloud = new float[4][point_count];

        for (int i = 0; i < point_count; i++) {
            byte[] Byterange = Arrays.copyOfRange(fileBytes, PointCloudlenght * i + Headerlength, PointCloudlenght * i + Headerlength + 4);
            float range = ByteBuffer.wrap(Byterange).order(ByteOrder.LITTLE_ENDIAN).getFloat();

            byte[] Byteazimuth = Arrays.copyOfRange(fileBytes, PointCloudlenght * i + Headerlength + 4, PointCloudlenght * i + Headerlength + 8);
            float azimuth = ByteBuffer.wrap(Byteazimuth).order(ByteOrder.LITTLE_ENDIAN).getFloat();

            byte[] Byteelevation = Arrays.copyOfRange(fileBytes, PointCloudlenght * i + Headerlength + 8, PointCloudlenght * i + Headerlength + 12);
            float elevation = ByteBuffer.wrap(Byteelevation).order(ByteOrder.LITTLE_ENDIAN).getFloat();

            byte[] Bytedoppler = Arrays.copyOfRange(fileBytes, PointCloudlenght * i + Headerlength + 12, PointCloudlenght * i + Headerlength + 16);
            float doppler = ByteBuffer.wrap(Bytedoppler).order(ByteOrder.LITTLE_ENDIAN).getFloat();

            point_cloud[0][i] = (float) (range * Math.cos(elevation) * Math.sin(azimuth));
            point_cloud[1][i] = (float) (range * Math.cos(elevation) * Math.cos(azimuth));
            point_cloud[2][i] = (float) (range * Math.sin(elevation));
        }
        point_cloud[3][0] = 0;

        return point_cloud;
    }

    private int sumArray(int[] array) {
        int sum = 0;

        for (int i : array)
            sum += i;
        return sum;
    }

    private double sumArrayDouble(double[] array) {
        double sum = 0.0;

        for (double i : array)
            sum += i;
        return sum;
    }

    private int sumArrayRange(int[] array, int start, int end) {
        int sum = 0;

        for (int i = start; i <= end; i++)
            sum += array[i];
        return sum;
    }

    private int geTLVType(byte[] fileBytes, int TLV_position) {
        // TLV object type
        int TLV_type = Arrays.copyOfRange(fileBytes, TLV_position, TLV_position + NIBBLE)[0];
        return TLV_type;
    }

    private int geTLVLength(byte[] fileBytes, int TLV_position) {
        // TLV object length, including TLV header length
        int TLV_Length = ByteArray2Int(Arrays.copyOfRange(fileBytes,
                TLV_position + NIBBLE, TLV_position + (NIBBLE * 2)));
        return TLV_Length;
    }


    private static byte[] stdOutputToByte(String output) {
        String[] data = output.trim().split(" ");
        int dataLength = data.length;
        byte[] byteData = new byte[dataLength];

        for (int i = 0; i < dataLength; i++) {
            try {
                byteData[i] = (byte) Integer.parseInt(data[i]);
            } catch (NumberFormatException e) {
                Log.d(ClassName, "NumberFormatException " + e);
            }
        }
        return byteData;
    }

    public static byte[] listByteToByteArray(List<Byte> data) {
        byte[] dataByte = new byte[data.size()];
        for (int i = 0; i < data.size(); i++) {
            dataByte[i] = data.get(i);
        }
        return dataByte;
    }

    ByteBuffer floatBuffer2byteBuffer(float flbuf[], int len) {
        ByteBuffer buffer = ByteBuffer.allocateDirect(len * 4);
        buffer.order(ByteOrder.nativeOrder());
        buffer.rewind();
        for (int i = 0; i < len; i++) {
            buffer.putFloat(flbuf[i]);
        }
        return buffer;
    }

    public static int ByteArray2Int(byte[] b) {
        int MASK = 0xFF;
        int result = 0;
        result = b[0] & MASK;
        result = result + ((b[1] & MASK) << 8);
        result = result + ((b[2] & MASK) << 16);
        result = result + ((b[3] & MASK) << 24);
        return result;
    }

    public static int ByteArray2Int_two(byte[] b) {
        int MASK = 0xFF;
        int result = 0;
        result = b[0] & MASK;
        result = result + ((b[1] & MASK) << 8);
        //result = result + ((b[2] & MASK) << 16);
        //result = result + ((b[3] & MASK) << 24);
        return result;
    }

    public static byte[] addBytes(byte[] data1, byte[] data2) {
        byte[] data3 = new byte[data1.length + data2.length];
        System.arraycopy(data1, 0, data3, 0, data1.length);
        System.arraycopy(data2, 0, data3, data1.length, data2.length);
        return data3;

    }
    // Load model by pathname
    private MappedByteBuffer loadModelFile(String model) throws IOException {
        AssetFileDescriptor fileDescriptor = getApplicationContext().getAssets().openFd(model + ".tflite");
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    float[] flatteninput(float stack_pixel[][]) {
        float[] input = new float[9 * 50 * 30];
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < (50 * 30); j++) {
                input[(i * 50 * 30) + j] = stack_pixel[i][j];
            }
        }
        return input;
    }

    float FindProb(float probArray[][]) {
        float temp = probArray[0][0];
        int key = 0;

        for (int count = 1; count < 7; count++) {
            if (temp < probArray[0][count]) {
                temp = probArray[0][count];
                key = count;
            }
        }
        return probArray[0][key];
    }

    int getOutputLabelindex(float probArray[][]) {
        float temp = probArray[0][0];
        int key = 0;

        for (int count = 1; count < 7; count++) {
            if (temp < probArray[0][count]) {
                temp = probArray[0][count];
                key = count;
            }
        }
        Log.d(ClassName, "numlabel:" + key);
        return key;
    }

    String FindProbIndex(float probArray[][]) {
        String[] classes = {"stand_walk", "st_sit", "sit_st", "sit_lie", "lie_sit", "fall", "get_up"};
        float temp = probArray[0][0];
        int key = 0;

        for (int count = 1; count < 7; count++) {
            if (temp < probArray[0][count]) {
                temp = probArray[0][count];
                key = count;
            }
        }
        System.out.println(probArray[0][key]);
        return classes[key];
    }

    float[][][] stackSlid_pixel(float pixel[][], float stack_pixel[][][], int frame_count) {
        if (frame_count < 9) {
            stack_pixel[0][frame_count] = pixel[0];
            stack_pixel[1][frame_count] = pixel[1];
        } else {
            float[][][] new_stack_pixel = new float[2][9][50 * 30];
            for (int i = 0; i < 8; i++) {
                new_stack_pixel[0][i] = stack_pixel[0][i + 1];  //third dim is for [X*Y points]
                new_stack_pixel[1][i] = stack_pixel[1][i + 1];
            }
            new_stack_pixel[0][8] = pixel[0];
            new_stack_pixel[1][8] = pixel[1];

            stack_pixel = new_stack_pixel;
        }

        return stack_pixel;
    }

    float[][] voxalize(float x[], float y[], float z[], int pointX, int pointY, int pointZ) {
        int len = x.length;

//        float [] pixel1 = new float[pointX * pointY];
//        float [] pixel2 = new float[pointY * pointZ];
        float[][] pixel = new float[2][pointX * pointZ];

        int x_min = -3;
        int x_max = 3;

        int y_min = 0;
        double y_max = 2.5;

        int z_max = 3;
        int z_min = -3;


        double x_res = (double) (x_max - x_min) / (pointX);
        double y_res = (double) (y_max - y_min) / pointY;
        double z_res = (double) (z_max - z_min) / pointZ;


        for (int i = 0; i < len; i++) {

            double x_pix = Math.floor((x[i] - (double) (x_min)) / x_res);
            double y_pix = Math.floor((y[i] - (double) (y_min)) / y_res);
            double z_pix = Math.floor((z[i] - (double) (z_min)) / z_res);


            if (x_pix > pointX) {
                continue;
            }
            if (y_pix > pointY) {
                continue;
            }
            if (z_pix > pointZ) {
                continue;
            }

            if (x_pix == pointX) {
                x_pix = pointX - 1;
            }
            if (y_pix == pointY) {
                y_pix = pointY - 1;
            }
            if (z_pix == pointZ) {
                z_pix = pointZ - 1;
            }

            int countx = (int) ((y_pix) * (pointX) + x_pix);
            int county = (int) ((y_pix) * (pointZ) + z_pix);

            if (countx > 0 && county > 0) {
                pixel[0][countx] += 1;
                pixel[1][county] += 1;
            } else {
                continue;
            }


        }
        return pixel;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.d(ClassName, "onDestroy()");
    }

}
