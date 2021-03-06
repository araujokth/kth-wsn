/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'TokenRequestMsg'
 * message type.
 */

public class TokenRequestMsg extends net.tinyos.message.Message {

    /** The default size of this message type in bytes. */
    public static final int DEFAULT_MESSAGE_SIZE = 6;

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 14;

    /** Create a new TokenRequestMsg of size 6. */
    public TokenRequestMsg() {
        super(DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /** Create a new TokenRequestMsg of the given data_length. */
    public TokenRequestMsg(int data_length) {
        super(data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TokenRequestMsg with the given data_length
     * and base offset.
     */
    public TokenRequestMsg(int data_length, int base_offset) {
        super(data_length, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TokenRequestMsg using the given byte array
     * as backing store.
     */
    public TokenRequestMsg(byte[] data) {
        super(data);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TokenRequestMsg using the given byte array
     * as backing store, with the given base offset.
     */
    public TokenRequestMsg(byte[] data, int base_offset) {
        super(data, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TokenRequestMsg using the given byte array
     * as backing store, with the given base offset and data length.
     */
    public TokenRequestMsg(byte[] data, int base_offset, int data_length) {
        super(data, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TokenRequestMsg embedded in the given message
     * at the given base offset.
     */
    public TokenRequestMsg(net.tinyos.message.Message msg, int base_offset) {
        super(msg, base_offset, DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TokenRequestMsg embedded in the given message
     * at the given base offset and length.
     */
    public TokenRequestMsg(net.tinyos.message.Message msg, int base_offset, int data_length) {
        super(msg, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
    /* Return a String representation of this message. Includes the
     * message type name and the non-indexed field values.
     */
    public String toString() {
      String s = "Message <TokenRequestMsg> \n";
      try {
        s += "  [criticalTime=0x"+Long.toHexString(get_criticalTime())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [tokenId=0x"+Long.toHexString(get_tokenId())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: criticalTime
    //   Field type: long, unsigned
    //   Offset (bits): 0
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'criticalTime' is signed (false).
     */
    public static boolean isSigned_criticalTime() {
        return false;
    }

    /**
     * Return whether the field 'criticalTime' is an array (false).
     */
    public static boolean isArray_criticalTime() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'criticalTime'
     */
    public static int offset_criticalTime() {
        return (0 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'criticalTime'
     */
    public static int offsetBits_criticalTime() {
        return 0;
    }

    /**
     * Return the value (as a long) of the field 'criticalTime'
     */
    public long get_criticalTime() {
        return (long)getUIntBEElement(offsetBits_criticalTime(), 32);
    }

    /**
     * Set the value of the field 'criticalTime'
     */
    public void set_criticalTime(long value) {
        setUIntBEElement(offsetBits_criticalTime(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'criticalTime'
     */
    public static int size_criticalTime() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'criticalTime'
     */
    public static int sizeBits_criticalTime() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: tokenId
    //   Field type: int, unsigned
    //   Offset (bits): 32
    //   Size (bits): 16
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'tokenId' is signed (false).
     */
    public static boolean isSigned_tokenId() {
        return false;
    }

    /**
     * Return whether the field 'tokenId' is an array (false).
     */
    public static boolean isArray_tokenId() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'tokenId'
     */
    public static int offset_tokenId() {
        return (32 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'tokenId'
     */
    public static int offsetBits_tokenId() {
        return 32;
    }

    /**
     * Return the value (as a int) of the field 'tokenId'
     */
    public int get_tokenId() {
        return (int)getUIntBEElement(offsetBits_tokenId(), 16);
    }

    /**
     * Set the value of the field 'tokenId'
     */
    public void set_tokenId(int value) {
        setUIntBEElement(offsetBits_tokenId(), 16, value);
    }

    /**
     * Return the size, in bytes, of the field 'tokenId'
     */
    public static int size_tokenId() {
        return (16 / 8);
    }

    /**
     * Return the size, in bits, of the field 'tokenId'
     */
    public static int sizeBits_tokenId() {
        return 16;
    }

}
