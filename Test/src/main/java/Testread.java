
import org.ujmp.core.Matrix;

import java.io.*;

public class Testread {

    public static void OutCsv(Matrix a,String outPutPath,String filename){
        File csvFile = null;
        BufferedWriter csvWtriter = null;
        try {
            csvFile = new File(outPutPath + File.separator + filename + ".csv");
            File parent = csvFile.getParentFile();
            if (parent != null && !parent.exists()) {
                parent.mkdirs();
            }
            csvFile.createNewFile();

            // GB2312使正确读取分隔符","
            csvWtriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(
                    csvFile), "GB2312"), 1024);
            // 写入文件头部
            for(int i = 0 ; i < a.getRowCount();i ++) {
                String head[] = new String[(int)a.getColumnCount()];
                for(int j = 0; j < a.getColumnCount(); j ++) {
                    head[j] = Double.toString(a.getAsDouble(i, j));
                }
                writeRow(head, csvWtriter);
            }

            csvWtriter.flush();
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                csvWtriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private static void writeRow(String[] row, BufferedWriter csvWriter) throws IOException {
        // 写入文件头部
        for (String data : row) {
            StringBuffer sb = new StringBuffer();
            String rowStr = sb.append("\"").append(data).append("\",").toString();
            csvWriter.write(rowStr);
        }
        csvWriter.newLine();
    }

}