/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'ControlMsg'
 * message type.
 */

public class ControlMsg extends net.tinyos.message.Message {

    /** The default size of this message type in bytes. */
    public static final int DEFAULT_MESSAGE_SIZE = 5;

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 17;

    /** Create a new ControlMsg of size 5. */
    public ControlMsg() {
        super(DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /** Create a new ControlMsg of the given data_length. */
    public ControlMsg(int data_length) {
        super(data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new ControlMsg with the given data_length
     * and base offset.
     */
    public ControlMsg(int data_length, int base_offset) {
        super(data_length, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new ControlMsg using the given byte array
     * as backing store.
     */
    public ControlMsg(byte[] data) {
        super(data);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new ControlMsg using the given byte array
     * as backing store, with the given base offset.
     */
    public ControlMsg(byte[] data, int base_offset) {
        super(data, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new ControlMsg using the given byte array
     * as backing store, with the given base offset and data length.
     */
    public ControlMsg(byte[] data, int base_offset, int data_length) {
        super(data, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new ControlMsg embedded in the given message
     * at the given base offset.
     */
    public ControlMsg(net.tinyos.message.Message msg, int base_offset) {
        super(msg, base_offset, DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new ControlMsg embedded in the given message
     * at the given base offset and length.
     */
    public ControlMsg(net.tinyos.message.Message msg, int base_offset, int data_length) {
        super(msg, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
    /* Return a String representation of this message. Includes the
     * message type name and the non-indexed field values.
     */
    public String toString() {
      String s = "Message <ControlMsg> \n";
      try {
        s += "  [cmd=0x"+Long.toHexString(get_cmd())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [u=";
        for (int i = 0; i < 1; i++) {
          s += Float.toString(getElement_u(i))+" ";
        }
        s += "]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: cmd
    //   Field type: short
    //   Offset (bits): 0
    //   Size (bits): 8
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'cmd' is signed (true).
     */
    public static boolean isSigned_cmd() {
        return true;
    }

    /**
     * Return whether the field 'cmd' is an array (false).
     */
    public static boolean isArray_cmd() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'cmd'
     */
    public static int offset_cmd() {
        return (0 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'cmd'
     */
    public static int offsetBits_cmd() {
        return 0;
    }

    /**
     * Return the value (as a short) of the field 'cmd'
     */
    public short get_cmd() {
        return (short)getUIntBEElement(offsetBits_cmd(), 8);
    }

    /**
     * Set the value of the field 'cmd'
     */
    public void set_cmd(short value) {
        setUIntBEElement(offsetBits_cmd(), 8, value);
    }

    /**
     * Return the size, in bytes, of the field 'cmd'
     */
    public static int size_cmd() {
        return (8 / 8);
    }

    /**
     * Return the size, in bits, of the field 'cmd'
     */
    public static int sizeBits_cmd() {
        return 8;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: u
    //   Field type: float[]
    //   Offset (bits): 8
    //   Size of each element (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'u' is signed (true).
     */
    public static boolean isSigned_u() {
        return true;
    }

    /**
     * Return whether the field 'u' is an array (true).
     */
    public static boolean isArray_u() {
        return true;
    }

    /**
     * Return the offset (in bytes) of the field 'u'
     */
    public static int offset_u(int index1) {
        int offset = 8;
        if (index1 < 0 || index1 >= 1) throw new ArrayIndexOutOfBoundsException();
        offset += 0 + index1 * 32;
        return (offset / 8);
    }

    /**
     * Return the offset (in bits) of the field 'u'
     */
    public static int offsetBits_u(int index1) {
        int offset = 8;
        if (index1 < 0 || index1 >= 1) throw new ArrayIndexOutOfBoundsException();
        offset += 0 + index1 * 32;
        return offset;
    }

    /**
     * Return the entire array 'u' as a float[]
     */
    public float[] get_u() {
        float[] tmp = new float[1];
        for (int index0 = 0; index0 < numElements_u(0); index0++) {
            tmp[index0] = getElement_u(index0);
        }
        return tmp;
    }

    /**
     * Set the contents of the array 'u' from the given float[]
     */
    public void set_u(float[] value) {
        for (int index0 = 0; index0 < value.length; index0++) {
            setElement_u(index0, value[index0]);
        }
    }

    /**
     * Return an element (as a float) of the array 'u'
     */
    public float getElement_u(int index1) {
        return (float)getFloatElement(offsetBits_u(index1), 32);
    }

    /**
     * Set an element of the array 'u'
     */
    public void setElement_u(int index1, float value) {
        setFloatElement(offsetBits_u(index1), 32, value);
    }

    /**
     * Return the total size, in bytes, of the array 'u'
     */
    public static int totalSize_u() {
        return (32 / 8);
    }

    /**
     * Return the total size, in bits, of the array 'u'
     */
    public static int totalSizeBits_u() {
        return 32;
    }

    /**
     * Return the size, in bytes, of each element of the array 'u'
     */
    public static int elementSize_u() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of each element of the array 'u'
     */
    public static int elementSizeBits_u() {
        return 32;
    }

    /**
     * Return the number of dimensions in the array 'u'
     */
    public static int numDimensions_u() {
        return 1;
    }

    /**
     * Return the number of elements in the array 'u'
     */
    public static int numElements_u() {
        return 1;
    }

    /**
     * Return the number of elements in the array 'u'
     * for the given dimension.
     */
    public static int numElements_u(int dimension) {
      int array_dims[] = { 1,  };
        if (dimension < 0 || dimension >= 1) throw new ArrayIndexOutOfBoundsException();
        if (array_dims[dimension] == 0) throw new IllegalArgumentException("Array dimension "+dimension+" has unknown size");
        return array_dims[dimension];
    }

}
