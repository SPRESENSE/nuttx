CXD5602 Audio Driver Design Document
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  CXD5602 has one spkeaker out, two I2S In/Out and one microphone
  input(max 8ch).
  And there are 3 DMA BUS Masters, for I2S0(in and out), for I2S1(in and out)
  and for micropohe.
  I2S0 and I2S1 DMA has 2 channels. One is for input(to memory), and the other
  is for output(from memory). These channels are shareing one DMA Master.
  So CXD5602 has 5 DMA channels for audio totaly.
  And it has also BEEP generator.
  CXD5602 Audio HW data path is described User Manual which can download from
  https://www.sony-semicon.com/files/62/pdf/p-28_CXD5602_user_manual.pdf in
  section 3.15.
  The figure below is logical data path diagram of this audio driver.


  MIC-DMA-Ch   DigitalIN-DMA-Ch    DigialOUT-DMA-Ch         SPK-DMA-Ch
      ^                ^                   |                    |
      |                |           +-------+                    |
      |                |           |                            |
      |                |           V                         +------+
      |     +------+   |       +------+                      |Volume|
      +---->|2ChSel|---------->|Select|                      +~~~~~~+
      |     +------+   |       +~~~~~~+                         |
      |                |           |                            V
      |                |           |                         +-----+
      |                +------------------+                +>|Mixer|
      |                |           |      |                | +~~~~~+
      |                |           |      V                |    |
      |                |           |   +------+   +------+ |    |
      |                |           +-->|Select|-->|Volume|-+    +---+
   +------+            |           |   +------+   +~~~~~~+          |
   |Volume|            |           |                                V
   +~~~~~~+            |           |     +--------+   +------+   +-----+
      ^                |           |     |Beep Gen|-->|Volume|-->|Mixer|
      |                |           |     +--------+   +------+   +-----+
      |                |           |                                |
      |                |           V                                V
 +---------+      +--------------------+                   +-----------+
 |MIC Input|      |input   I2S   output|                   |Speaker Out|
 +~~~~~~~~~+      +~~~~~~~~~~~~~~~~~~~~+                   +~~~~~~~~~~~+


  There is a mixer to mix 2 audio to speaker out, and one of the path is
  shared DigialIN and DigitalOUT.

  To I2S output, MIC input can be selected instead of DMA channel.
  And this path is selected as Speaker output too. This means DigitalOut
  and MIC input are affecting for each other.

  Speaker DMA ch, Digital IN DMA ch and Digital OUT DMA ch have an
  limitation about sampling rate.
  This audio system has 2 sampling rate mode, which are 48KHz and 192KHz.
  These 3 DMA channels can select the same sampling rate, not indipendently.
  So if any channels is set 192KHz for example, other channels should
  follow it.
  So the first device set sampling rate is set that is valid for the set
  value. Therefore, if another device sets a different set value later, an
  error will result.
  MIC DMA is not affected on this limitation. MIC DMA can be independent
  for it.

Audio Device Files
~~~~~~~~~~~~~~~~~~

  As the logical block diagram above, this driver uses 4 audio DMA channels.
  And device files are created for each channels.

    MIC-DMA-Ch        => /dev/pcm_in0
    SPK-DMA-Ch        => /dev/pcm0
    DigialIN-DMA-Ch   => /dev/pcm_in1
    DigitalOUT-DMA-Ch => /dev/pcm1

  And beep device is also defined

    Beep device       => /dev/beep0


Audio Device Distination Select
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  The path of MIC input and I2S input have some distinations
  which are to memory, to speaker and to both memory and speaker.
  Therefore, the concept of a "distination" is established for each path.

    - DISTINATION_MEM
    - DISTINATION_SPK
    - DISTINATION_BOTH

  DMA works when the distination is selected including to memory.

  And because of the sharing path to speaker from DigialIN, DigialOUT and,
  Mic input, these are exclusive to use speaker out. The one who selects
  the Speaker output first can use the resources of the Speaker output,
  and the one who selects it later will return an error.

  Within this hardware constraint, it is very complex to specify each DMA
  channel with its own independence and abstraction.
  So at this time, direct output to Speaker is not supported.


Audio Driver Archiechture
~~~~~~~~~~~~~~~~~~~~~~~~~

  The methods to control the driver is followed nuttx audio lower driver
  framework. And the data comming to/from memory is exchanged by exchanging
  the ap_buffer_t data block.
  An application, which is a user of the device, enqueue the ap_buffer_t
  data blocks, and then sequential DMA is performed. To store the buffer
  which is waiting for execute DMA, a dq_queue named dma_pendq is used.
  And one more dq_queue named dma_runq is used to store the buffer which
  is waiting for finishing DMA.

  After finished the DMA, dma interrupt handler pulls a ap_buffer from
  dq_queue which is for waiting for finishing DMA, and the ap_buffer is
  released to audio framework. And then the handler pulls a ap_buffer from
  dq_queue which is for waiting for execute DMA, and starts DMA for the
  pulled ap_buffer. and the ap_buffer itself is pushed into the dq_queue
  which is for waiting for finishing that.

                           dma_pendq
                            -------+
  ENQUEUE <ap_buffer> ---->        |-------------------+
                            -------+                   |
                                                       |
                                 +---------------------|------------+
                                 | DMA handler         V            |
                                 |                pull new          |
                                 |                ap_buffer         |
  DEQUEUE event <-------------------Release         |   |           |
  via message queue              |  ap_buffer    +--+   +----+      |
                                 |      ^        |           |      |
                                 +------|--------V-----------|------+
                                        |   | DMA HW |       |
                                        |   +--------+       |
                                        |                    |
                                        |   dma_runq        push
                                        |   +------          |
                                        +---|       <--------+
                                            +------

  Each DMA channels related on each device files have this structure.


Audio Driver State
~~~~~~~~~~~~~~~~~~

  To cotrol each devices and its DMA, 9 states are defined in this driver.

  State       Description
  ----------  ---------------------------------------------------------------
  INITIAL     Initialized (Actualy Audio driver has no method related on
                           open(). This state should be entered after open())
  CONFIGURED  Configured with valid paramter
  STARTING    DMA start is requested but not enough dma buffer is stored
  STARTED     DMA is running
  PAUSED      DMA is just stopped.
  STOPPING    DMA stop is requested and waiting for final DMA finished.
  CLOSING     close() is called and wait for final DMA finished.
  PAUSING     pause() is called and wait for stop DMA.
  COMPLETING  received APB buffer with APB_FINAL flag and wait for final DMA.


  === State Transition Chart === (LEGEND: <...> Condition to transit)

  +---------------- <Final DMA finished> ------------------------+
  |                                                              |
  | <reserve()>                                                  |
  |    ___   +-------------- <configure()> -------------------+  |
  |    | |   |               & <non-valid param>              |  |
  |    | v   V                                                |  v
  |    INITIAL -- <configure()> ---------------------------> CONFIGURED <----+ <-----+
  |              & <vaild param>                                  |          |       |
  |                                     +-------------------------+          |       |
  |                                     |                         |          |       |
  |                                <start()> &               <start()> &     |       |
  |                               <enough buf>             <not enough buf>  |       |
  |                                     |                         |          |       |
  |                                     v                         v          |       |
  +-- STOPPING <-- <stop()> -------- STARTED <- <enqueue()> ---STARTING      |       |
          |         or <DMA Error>    |  | |   & <enough buf>                |       |
    <shutdown()>    or <UnderRUN>     |  | |                   <Final DMA finished>  |
     or <release()>                   |  | |                                 |       |
          |                           |  | +-<AUDIO_APB_FINAL>-->COMPLETING -+       |
          v                           |  |                           |               |
       CLOSING <--- <shutdown()> -----+<-----------------------------+               |
          |         or <release()>       |                                           |
          |                  ^       <pause()>                                       |
          |                  |           |                                           |
   <Final DMA finished>      |           V                                           |
          |                  +--------PAUSING---- <stop()> --> STOPPING              |
          |                              |                                           |
          v                              |                                           |
        INITIAL                     <Final DMA finished>                             |
 (then shutdown()/release()              |                                           |
  is returned)                           | +------ <resume()> -------> STARTING      |
                                         | |    & <no enough buf>                    |
                                         v |                                         |
                                      PAUSED -- <resume()> -------> STARTED          |
                                         |    & <enouch buf>                         |
                                         |                                           |
                                         +-- <stop()> -------------------------------+

  shutdown() or release() makes all Status to go to INITIAL except STARTED and
  XXX-ING states. configure() is not acceptable after START.
  On this driver, reserve() method is no meaning, just return OK.


  === Details of transitions and actions =======

  <Current State> |  <Transition Event>        <Next State>   <Action(s)>
  ====================================================================================================
  INITIAL         |  ioctl(CONFIG) &           CONFIGURED     If the Audio system is powered off,
                  |  valid parameters                         make it turn on, otherwise, do nothing.
                  +-----------------------------------------------------------------------------------
                  |  reserve()                 INITIAL        If the Audio system is powered off,
                  |                                           make it turn on, otherwise, do nothing.
                  +-----------------------------------------------------------------------------------
                  |  shutdown()                INITIAL        Initialize the instance.
                  |                                           And if no more opened device file,
                  |                                           turn off audio system.
  ----------------+-----------------------------------------------------------------------------------
  CONFIGURED      |  ioctl(CONFIG) &           INITIAL        Do nothing
                  |  non-valid parameters
                  +-----------------------------------------------------------------------------------
                  |  start() &                 STARTED        DMA Start with full DMA fifo.
                  |  enough apb in pendq                      Set volume.
                  +-----------------------------------------------------------------------------------
                  |  start() &                 STARTING       Do nothing
                  |  not enouch apb in pendq
                  +-----------------------------------------------------------------------------------
                  |  shutdown()                INITIAL        Initialize the instance.
                  |                                           And if no more opened device file,
                  |                                           turn off audio system.
  ----------------+-----------------------------------------------------------------------------------
  STARTING        |  ioctl(ENQUEUE) &          STARTING       Do nothing.
                  |  not enough apb in pendq
                  +-----------------------------------------------------------------------------------
                  |  ioctl(ENQUEUE) &          STARTED        DMA Start with full DMA fifo.
                  |  enough apb in pendq                      Set volume.
                  +-----------------------------------------------------------------------------------
                  |  pause()                   PAUSED         Do nothing.
                  +-----------------------------------------------------------------------------------
                  |  stop()                    CONFIGURED     Dequeue all apb buffers in pendq.
                  +-----------------------------------------------------------------------------------
                  |  shutdown()                INITIAL        Initialize the instance.
                  |                                           And if no more opened device file,
                  |                                           turn off audio system.
  ----------------+-----------------------------------------------------------------------------------
  STARTED         |  stop()                    STOPPING       Set volume minimum(Not mute).
                  |                                           Store DMA stop command in DMA fifo.
                  |                                           Caller will wait until receiving DMA
                  |                                           is finished.
                  +-----------------------------------------------------------------------------------
                  |  DMA done &                CONFIGURED     EIO Error callback is called.
                  |  Error is occured                         Store DMA stop command in DMA fifo.
                  |                                           Interrupt is masked.
                  +-----------------------------------------------------------------------------------
                  |  DMA done &                STOPPING       Set volume minium(Not mute).
                  |  pendq is empty &                         Underrun callback is called.
                  |  The space of HW fifo is                  Store DMA stop command in DMA fifo.
                  |  less than 2
                  +-----------------------------------------------------------------------------------
                  |  DMA done &                COMPLETING     Set volume minimun(Not mute)
                  |  APB from pendq has FINAL                 Store DMA stop command in DMA fifo.
                  |  flag.
                  +-----------------------------------------------------------------------------------
                  |  DMA done &                STARTED(stay)  DEQUEUE buffer from runq.
                  |  No condition above.                      Set new APB from pendq to runq.
                  |                                           And set DMA command for the new APB.
                  +-----------------------------------------------------------------------------------
                  |  pause()                   PAUSING        Store DMA stop command in DMA fifo.
                  |                                           Set volume minimum(Not mute).
                  |                                           Caller will wait until receiving DMA
                  |                                           is finished.
                  +-----------------------------------------------------------------------------------
                  |  shutdown() or             CLOSING        Store DMA stop command in DMA fifo.
                  |  release()                                Set volume minimum(Not mute).
                  |                                           Caller will wait until receiving DMA
                  |                                           is finished.
  ----------------+-----------------------------------------------------------------------------------
  STOPPING        |  DMA Done &                CONFIGURED     Callback STOP message.
                  |  The DMA was final                        Set volume Mute.
                  |                                           Dequeue all APBs in runq and pendq.
                  |                                           Interrupt is masked.
                  |                                           If there is waiting caller, send message
                  |                                           to it.
                  +-----------------------------------------------------------------------------------
                  |  DMA Done &                STOPPING       Dequeue callback is called with a APB
                  |  Not final and no error    (Stay)         in top of runq.
                  +-----------------------------------------------------------------------------------
                  |  shutdown() or             CLOSING        Do nothing.
                  |  release()
  ----------------+-----------------------------------------------------------------------------------
  COMPLETING      |  DMA Done &                CONFIGURED     Callback Complete message.
                  |  The DMA was final                        Set volume Mute.
                  |                                           Dequeue all APBs in runq and pendq.
                  |                                           Interrupt is masked.
                  +-----------------------------------------------------------------------------------
                  |  shutdown() or             CLOSING        Do nothing.
                  |  release()
  ----------------+-----------------------------------------------------------------------------------
  PAUSING         |  DMA Done &                PAUSED         Callback PAUSE message.
                  |  The DMA was final                        Set volume Mute.
                  |                                           Dequeue all APBs in runq.
                  |                                           Interrupt is masked.
                  +-----------------------------------------------------------------------------------
                  |  shutdown() or             CLOSING        Do nothing.
                  |  release()
  ----------------+-----------------------------------------------------------------------------------
  PAUSED          |  resume() or               STARTED        DMA Start with full DMA fifo.
                  |  start() &                                Set volume.
                  |  enough apb buffers
                  +-----------------------------------------------------------------------------------
                  |  resume() or               STARTING       Do nothing.
                  |  release() &
                  |  not enough apb buffers
                  +-----------------------------------------------------------------------------------
                  |  stop()                    CONFIGURED     Do nothing.
                  +-----------------------------------------------------------------------------------
                  |  shutdown() or             INITIAL        Initialize the instance.
                  |  release()
  ----------------+-----------------------------------------------------------------------------------
  CLOSING         |  DMA Done &                INITIAL        Callback STOP message.
                  |  The DMA was final                        Set volume Mute.
                  |                                           Dequeue all APBs in runq and pendq.
                  |                                           Interrupt is masked.
                  |                                           If there is waiting caller, send message
                  |                                           to it.
  ====================================================================================================



CXD5602 Audio Lower
~~~~~~~~~~~~~~~~~~~

  CXD5602 Audio Driver has lower layer which is to abstruct each board specific
  external hardware.

  The lower layer is provided methods to implement functions for abstructing the
  board specific with following 'struct cxd56_audio_lower_s'.
  The structure has

    chmaps     : returns audio mic channel maps.
    clksrc     : returns select of audio clock source.
    clkdif     : returns clock divide value if clksrc() returned non-MCLK_EXT.
    is_mclk24m : returns true if external clock source is 24.576MHz.
    ext_mute   : controls external mute circuit, if it has.
    ext_micen  : controls external microphone power control circuit, if it has.

  These methods must provide by each board implementations.
