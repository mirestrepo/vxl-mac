#include <vcl_fstream.h>



#include <vcl_functional.h>
#include <vcl_deque.h>

#include <mbl/mbl_priority_bounded_queue.h>
#include <testlib/testlib_test.h>

void test_priority_bounded_queue()
{
  vcl_cout << "\n\n************************************" << vcl_endl;
  vcl_cout << " Testing mbl_priority_bounded_queue " << vcl_endl;
  vcl_cout << "************************************" << vcl_endl;

  mbl_priority_bounded_queue<int, vcl_deque<int>, vcl_greater<int> > pq(3);

  pq.push(6);
  pq.push(4);
  pq.push(2);
  pq.push(0);

  TEST("Priority queue is size 3, after adding 4 elements", pq.size(), 3);
  TEST("Priority queue top() is correct ", pq.top(), 2);
  pq.pop();
  TEST("After 1 pop, Priority queue top() is correct ", pq.top(), 4);
  pq.pop();
  TEST("After 2 pops, Priority queue top() is correct ", pq.top(), 6 );
  pq.pop();
  TEST("After 3 pops, Priority queue now empty ", pq.size(), 0);

}

TESTLIB_DEFINE_MAIN(test_priority_bounded_queue);
