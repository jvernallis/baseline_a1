#include <verilated.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <csignal>
#include <cstring>
#include <unordered_map>
#include <unistd.h>
#include "Vmips_core.h"
#include "verilated_fst_c.h"
#include "Vmips_core__Dpi.h"
#include "memory_driver.h"
#include "memory.h"

Vmips_core *top; // Instantiation of module
MemoryDriver *memory_driver;
Memory *memory;

int memory_debug = 0;
int stream_dump = 0;
int stream_print = 0;
int stream_check = 1;
const char *benchmark[2] = {"nqueens", "coin"};
// std::string hexfiles_dir = "/home/linux/ieng6/cs148sp22/public";
std::string hexfiles_dir = "..";

vluint64_t main_time = 0; // Current simulation time
// This is a 64-bit integer to reduce wrap over issues and
// allow modulus.  This is in units of the timeprecision
// used in Verilog (or from --timescale-override)

double sc_time_stamp()
{                     // Called by $time in Verilog
    return main_time; // converts to double, to match
                      // what SystemC does
}

volatile std::sig_atomic_t interrupt = 0;
vluint64_t stop_time = 0;

void signal_handler(int signal)
{
    interrupt = signal;
}

std::map<std::string, unsigned int> stats;

void stats_event(const char *e)
{
    std::string s(e);
    stats[s]++;
}

unsigned int instruction_count = 0;

void pc_event(const int pc, const int thread_id)
{   
    // When a thread restarts, previously fetched cycles are re-fetched.
    // So, make an allowance for 3 cycles of error on thread switch
    static int last_thread_id;
    static int thread_restart_cycles = 0;
    static const int thread_restart_tolerance = 4; 

    static std::ofstream f_write[2];
    static std::ifstream f_read[2];
    const std::string fname[2] = {std::string(hexfiles_dir + "/hexfiles/"+ std::string(benchmark[0]) +".pc.txt"),
                                    std::string(hexfiles_dir + "/hexfiles/"+ std::string(benchmark[1]) +".pc.txt")};

    static bool initialized = false;

    if (!initialized) {
        for (int i = 0; i < 2; i++) {
            if(stream_dump)
                f_write[i].open(fname[i]);
            f_read[i].open(fname[i]);
            if((stream_dump && !f_write[i].is_open()) || !f_read[i].is_open()) {
                std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
                exit(-1);
            }
        }
        initialized = true;
    }

    if (stream_print)
        std::cout << "-- EVENT pc=" << std::hex << pc << std::endl;
    if (stream_dump)
    {
        if (!f_write[thread_id].is_open())
        {
            std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
            exit(-1);
        }
        if (stream_dump >= 2)
            f_write[thread_id] << std::dec << main_time << " ";
        f_write[thread_id] << std::hex << pc << std::endl;
    }
    if (stream_check)
    {
        if (!f_read[thread_id].is_open())
        {
            std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
            exit(-1);
        }

        unsigned int expected_pc;
        unsigned int clear_mask = 1 << (sizeof(unsigned int) * 8 - 7);
        unsigned int set_mask = thread_id << (sizeof(unsigned int) * 8 - 7);

        // If a thread switch occurred already fetched instructions will be fetched again.
        // Misses against the file are expected to occur.
        
        //If the thread is restarting, don't start reading from the file until the expected PC is matched again
        if((last_thread_id != thread_id) || (thread_restart_cycles != 0))
        {
            if(last_thread_id != thread_id){
                bool pc_valid = bool(f_read[thread_id] >> std::hex >> expected_pc);
                expected_pc = (expected_pc & ~clear_mask) ^ set_mask;
                if(!pc_valid){
                    std::cout << "\n!! Ran out of expected pc on a thread restart."
                            "\n!! More instructions are executed than expected."
                            "\n!! This was likely caused by the restart of a completed thread."
                            "\n!! Additional pc="
                            << std::hex << pc << std::endl;
                    std::cout << "When this error was raised, the thread ID was " << thread_id << std::endl;
                    std::raise(SIGINT);
                }
            } 
            

            thread_restart_cycles++;
            if(expected_pc == pc){
                thread_restart_cycles = 0;
            } else if (thread_restart_cycles > thread_restart_tolerance) {
                std::cout << "\n!! [" << std::dec << main_time << "] thread restart failed: last expected_pc=" << std::hex << expected_pc
                        << " mismatches pc=" << pc << std::endl;
                std::raise(SIGINT);
            }
        } else {
            bool pc_valid = bool(f_read[thread_id] >> std::hex >> expected_pc);
            expected_pc = (expected_pc & ~clear_mask) ^ set_mask;

            if (!pc_valid)
            {
                std::cout << "\n!! Ran out of expected pc."
                            "\n!! More instructions are executed than expected"
                            "\n!! Additional pc="
                        << std::hex << pc << std::endl;
                std::cout << "When this error was raised, the thread ID was " << thread_id << std::endl;
                std::raise(SIGINT);
            }
            else if (expected_pc != pc)
            {
                std::cout << "\n!! [" << std::dec << main_time << "] expected_pc=" << std::hex << expected_pc
                        << " mismatches pc=" << pc << std::endl;
                std::raise(SIGINT);
            }
        }
    }
    last_thread_id = thread_id;
    instruction_count++;
}

unsigned int write_back_count = 0;

void wb_event(const int addr, const int data, const int thread_id)
{
    static std::ofstream f_write[2];
    static std::ifstream f_read[2];
    const std::string fname[2] = {std::string(hexfiles_dir + "/hexfiles/"+ std::string(benchmark[0]) +".wb.txt"),
                                    std::string(hexfiles_dir + "/hexfiles/"+ std::string(benchmark[1]) +".wb.txt")};

    static bool initialized = false;

    if (!initialized) {
        for (int i = 0; i < 2; i++) {
            if(stream_dump)
                f_write[i].open(fname[i]);
            f_read[i].open(fname[i]);
            if((stream_dump && !f_write[i].is_open()) || !f_read[i].is_open()) {
                std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
                exit(-1);
            }
        }
        initialized = true;
    }

    if (stream_print)
        std::cout << "-- EVENT wb addr=" << std::hex << addr
                  << " data=" << data << std::endl;
    if (stream_dump)
    {
        if (!f_write[thread_id].is_open())
        {
            std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
            exit(-1);
        }
        if (stream_dump >= 2)
            f_write[thread_id] << std::dec << main_time << " ";
        f_write[thread_id] << std::hex << addr << " " << data << std::endl;
    }
    if (stream_check)
    {
        if (!f_read[thread_id].is_open())
        {
            std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
            exit(-1);
        }

        unsigned int expected_addr, expected_data;
        unsigned int clear_mask = 1 << (sizeof(unsigned int) * 8 - 7);
        unsigned int set_mask = thread_id << (sizeof(unsigned int) * 8 - 7);

        bool rw_valid = bool(f_read[thread_id] >> std::hex >> expected_addr >> expected_data);
        expected_addr = (expected_addr & ~clear_mask) ^ set_mask;
        if (!rw_valid)
        {
            std::cout << "\n!! Ran out of expected write back."
                         "\n!! More write back are executed than expected"
                         "\n!! Additional write back addr="
                      << std::hex << addr << " data=" << data << std::endl;
            std::raise(SIGINT);
        }
        // Experimental (kludge): Take out MSB of read data.
        else if (expected_addr != addr || (expected_data != data && (expected_data ^ set_mask) != data))
        {
            std::cout << "\n!! [" << std::dec << main_time << "] expected write back mismatches"
                      << "\n!! [" << std::dec << main_time << "] expected addr=" << std::hex << expected_addr
                      << " data=" << expected_data << " or " << (expected_data ^ clear_mask)
                      << "\n!! [" << std::dec << main_time << "] actual   addr=" << std::hex << addr
                      << " data=" << data << std::endl;
            std::raise(SIGINT);
        }
    }

    write_back_count++;
}

unsigned int load_store_count = 0;
void ls_event(const int op, const int addr, const int data, const int thread_id)
{
    static std::ofstream f_write[2];
    static std::ifstream f_read[2];
    const std::string fname[2] = {std::string(hexfiles_dir + "/hexfiles/"+ std::string(benchmark[0]) +".ls.txt"),
                                    std::string(hexfiles_dir + "/hexfiles/"+ std::string(benchmark[1]) +".ls.txt")};

    static bool initialized = false;

    if (!initialized) {
        for (int i = 0; i < 2; i++) {
            if(stream_dump)
                f_write[i].open(fname[i]);
            f_read[i].open(fname[i]);
            if((stream_dump && !f_write[i].is_open()) || !f_read[i].is_open()) {
                std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
                exit(-1);
            }
        }
        initialized = true;
    }

    if (stream_print)
        std::cout << "-- EVENT ls op=" << std::hex << op
                  << " addr=" << addr
                  << " data=" << data << std::endl;
    if (stream_dump)
    {
        std::string fname(hexfiles_dir + "/hexfiles/"+ std::string(benchmark[thread_id]) +".ls.txt");
        static std::ofstream f(fname);
        if (!f_write[thread_id].is_open())
        {
            std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
            exit(-1);
        }
        if (stream_dump >= 2)
            f_write[thread_id] << std::dec << main_time << " ";
        f_write[thread_id] << std::hex << op << " " << addr << " " << data << std::endl;
    }
    if (stream_check)
    {
        if (!f_read[thread_id].is_open())
        {
            std::cerr << "Failed to open file: " << fname[thread_id] << std::endl;
            exit(-1);
        }

        unsigned int expected_op, expected_addr, expected_data;
        unsigned int mask = thread_id<< (sizeof(unsigned int) * 8 - 7);
        unsigned int clear_mask = 1 << (sizeof(unsigned int) * 8 - 7);
        unsigned int set_mask = thread_id << (sizeof(unsigned int) * 8 - 7);

        bool rw_valid = bool(f_read[thread_id] >> std::hex >> expected_op && f_read[thread_id] >> expected_addr && f_read[thread_id] >> expected_data);
        expected_addr = (expected_addr & ~clear_mask) ^ set_mask;
        if (!rw_valid)
        {
            std::cout << "\n!! Ran out of expected load store"
                         "\n!! More load store are executed than expected"
                         "\n!! Additional load store op="
                      << std::hex << op << " addr=" << addr << " data=" << data << std::endl;
            std::raise(SIGINT);
        }
        // Experimental (kludge): Take out MSB of read data.
        else if (expected_op != op || expected_addr != addr || (expected_data != data && (expected_data ^ set_mask) != data))
        {
            std::cout << "\n!! [" << std::dec << main_time << "] expected load store mismatches"
                      << "\n!! [" << std::dec << main_time << "] expected op=" << std::hex << expected_op
                      << " addr=" << expected_addr
                      << " data=" << expected_data << " or " << (expected_data ^ set_mask)
                      << "\n!! [" << std::dec << main_time << "] actual   op=" << std::hex << op
                      << " addr=" << addr
                      << " data=" << data << std::endl;
            std::raise(SIGINT);
        }
    }

    load_store_count++;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);

    int opt;
    int dump = 0;
    double memory_delay_factor = 1.0;
    while ((opt = getopt(argc, argv, "dmpstf:b:c:")) != -1)
    {
        switch (opt)
        {
        case 'd':
            // Dump verilog waves to simx.fst
            dump = 1;
            break;
        case 'm':
            // Print debug info for cpp memory model
            // Repeat to increase verbose level
            memory_debug++;
            break;
        case 'p':
            // Print stream events to stdout
            stream_print = 1;
            break;
        case 's':
            // Skip stream checks
            stream_check = 0;
            break;
        case 't':
            // Trace streams and save to files
            // Repeat to include time in the trace
            stream_dump++;
            break;
        case 'f':
            // Set memory delay factor
            {
                std::stringstream argument(optarg);
                argument >> memory_delay_factor;
            }
            break;
        case 'b':
            benchmark[0] = optarg;
            break;
        case 'c':
            benchmark[1] = optarg;
            break;
        default: /* '?' */
            std::cerr << "Usage: " << argv[0] << " [-dmpst] [-b thread 0 benchmark] [-c thread 1 benchmark] [+plusargs]" << std::endl;
            return -1;
        }
    }

    if(!strcmp(benchmark[0], benchmark[1])) {
        std::cerr << "Running the same benchmark twice not yet supported." << std::endl;
        return -1;
    }

    Verilated::commandArgs(argc, argv); // Remember args

    top = new Vmips_core; // Create instance
    std::string hex_file_strings[2] = {(hexfiles_dir + "/hexfiles/" + std::string(benchmark[0]) + ".hex"),
                                        (hexfiles_dir + "/hexfiles/" + std::string(benchmark[1]) + ".hex")};
    const char* hex_file_names[2] = {hex_file_strings[0].c_str(), hex_file_strings[1].c_str()};

    memory = new Memory(hex_file_names, memory_delay_factor);
    memory_driver = new MemoryDriver(top, memory);

    VerilatedFstC *tfp;
    if (dump)
    {
        std::cout << "Dumping waveform to simx.fst\n";
        Verilated::traceEverOn(true);
        tfp = new VerilatedFstC;
        top->trace(tfp, 1024);
        tfp->open("simx.fst");
    }

    top->clk = 0;
    top->rst_n = 0;
    memory_driver->drive_reset();

    while (!top->done && !(interrupt && main_time >= stop_time))
    {
        top->clk = !top->clk; // Toggle clock
        if (top->clk)
            memory_driver->consume(main_time);
        if (main_time == 100)
            top->rst_n = 1; // Deassert reset
        top->eval();        // Evaluate model
        if (top->clk)
        {
            memory_driver->drive(main_time);
            memory->process(main_time);
        }
      //  if (main_time % 1000000 == 0)
        //    std::cout << "Time is now: " << main_time << std::endl;
        if (dump)
            tfp->dump(main_time);

        main_time += 5; // Time passes...

        if (interrupt && stop_time == 0)
        {
            stop_time = main_time + 1000;
            std::cerr << "\n!! Interrupt raised at time=" << main_time << std::endl
                      << "!! Running additional 100 cycles before terminating at stop_time=" << stop_time << std::endl;
        }
    }

    top->final(); // Done simulating
    delete memory_driver;
    delete memory;
    delete top;

    if (dump)
    {
        tfp->close();
    }

    int cycle_count = main_time / 10;
    std::cout << std::dec
              << "\n\nTotal time: " << main_time
              << "\nCycle count: " << cycle_count
              << "\nInstruction count: " << instruction_count
              << "\nCPI: " << (float)cycle_count / instruction_count << " IPC: " << (float)instruction_count / cycle_count << std::endl;

    std::cout << "\n== Stats ===============\n";

    for (const auto &e : stats)
        std::cout << e.first << ": " << e.second << std::endl;

    if (interrupt)
        std::cerr << "\n== ABORTED =============\nSimulation aborted at stop_time=" << main_time << std::endl;
}
