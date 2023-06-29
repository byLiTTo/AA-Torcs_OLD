package torcs;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.StringTokenizer;

/**
 * The MessageParser class is responsible for parsing messages received from the server bot in the TORCS (The Open
 * Racing Car Simulator) environment.
 * It creates a table of associated names and values for the readings extracted from the message.
 *
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Administrator</p>
 * <p>Date: Feb 22, 2008</p>
 * <p>Time: 6:17:32 PM</p>
 */
public class MessageParser {
    private Hashtable<String, Object> table = new Hashtable<String, Object>(); // The table of reading names and values
    private String message; // The original message received

    /**
     * Constructs a new MessageParser with the specified message.
     *
     * @param message The message to be parsed.
     */
    public MessageParser(String message) {
        this.message = message;
        StringTokenizer mt = new StringTokenizer(message, "(");
        while (mt.hasMoreElements()) {
            String reading = mt.nextToken();
            int endOfMessage = reading.indexOf(")");
            if (endOfMessage > 0) {
                reading = reading.substring(0, endOfMessage);
            }
            StringTokenizer rt = new StringTokenizer(reading, " ");
            if (rt.countTokens() < 2) {
                // Reading not recognized
            } else {
                String readingName = rt.nextToken();
                Object readingValue = "";
                if (readingName.equals("opponents") || readingName.equals("track") ||
                        readingName.equals("wheelSpinVel") || readingName.equals("focus")) {
                    // Readings with multiple values
                    readingValue = new double[rt.countTokens()];
                    int position = 0;
                    while (rt.hasMoreElements()) {
                        String nextToken = rt.nextToken();
                        try {
                            ((double[]) readingValue)[position] = Double.parseDouble(nextToken);
                        } catch (Exception e) {
                            System.out.println("Error parsing value '" + nextToken + "' for " + readingName + " using 0.0");
                            System.out.println("Message: " + message);
                            ((double[]) readingValue)[position] = 0.0;
                        }
                        position++;
                    }
                } else {
                    String token = rt.nextToken();
                    try {
                        readingValue = Double.parseDouble(token);
                    } catch (Exception e) {
                        System.out.println("Error parsing value '" + token + "' for " + readingName + " using 0.0");
                        System.out.println("Message: " + message);
                        readingValue = (Double) 0.0;
                    }
                }
                table.put(readingName, readingValue);
            }
        }
    }

    /**
     * Prints all the readings and their associated values in the table.
     */
    public void printAll() {
        Enumeration<String> keys = table.keys();
        while (keys.hasMoreElements()) {
            String key = keys.nextElement();
            System.out.print(key + ":  ");
            System.out.println(table.get(key));
        }
    }

    /**
     * Retrieves the value of a specific reading from the table.
     *
     * @param key The name of the reading.
     *
     * @return The value associated with the reading, or null if the reading is not found.
     */
    public Object getReading(String key) {
        return table.get(key);
    }

    /**
     * Retrieves the original message that was parsed.
     *
     * @return The original message as a string.
     */
    public String getMessage() {
        return message;
    }
}