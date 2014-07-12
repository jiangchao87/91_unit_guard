#ifndef __H_LISTHEAD__
  #define __H_LISTHEAD__

#include	"typedefine.h"

/*
 * Simple doubly linked list implementation.
 *
 * Some of the internal functions ("__xxx") are useful when
 * manipulating whole lists rather than single entries, as
 * sometimes we already know the next/prev entries and we can
 * generate better code by using them directly rather than
 * using the generic single-entry routines.
 */
 
// 表头
typedef volatile struct list_head 
{
	volatile struct list_head *next, *prev;
}list_head;												// 定义的结构体包含2个 指向同类型结构体的 指针

	
/*
 * list_entry - get the struct for this entry
 * @ptr:	the &struct list_head pointer.
 * @type:	the type of the struct this is embedded in.
 * @member:	the name of the list_struct within the struct.
 */
#define  list_entry(ptr, type, member) \
	( (type *)( (char *)(ptr) - (unsigned long)(&((type *)0)->member) ) )

// 链表 头 初始化
#define LIST_HEAD_INIT(name)  { &(name), &(name) }

// define name and then initial 
#define LIST_HEAD(name) \
	volatile struct list_head name = LIST_HEAD_INIT(name)	
// 定义 list_head类型的结构体 name 值等于 &(name), &(name)  即这个结构体里的2个元素都指向自己本身


// 初始化链表头  本函数的参数是 list_head类型的结构体指针 将这个指针指向的结构体的2个元素赋值为自己的结构体首地址
void INIT_LIST_HEAD(volatile struct list_head *list)
{
	list->next = list;
	list->prev = list;
}

// 增加一个节点
void __list_add(volatile struct list_head *new, volatile struct list_head *prev,volatile struct list_head *next)
{
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

/*
 * list_add - add a new entry
 * @new: new entry to be added
 * @head: list head to add it after
 *
 * Insert a new entry after the specified head.
 * This is good for implementing stacks.
 */
void list_add(volatile struct list_head *new, volatile struct list_head *head)
{
	__list_add(new, head, head->next);
}

// 将 prev和next2个结构体连起来  去除他们中间的内容
void __list_del(volatile struct list_head * prev, volatile struct list_head * next)
{
	next->prev = prev;
	prev->next = next;
}



/*
 * list_del - deletes entry from list.
 * @entry: the element to delete from the list.
 * Note: list_empty on entry does not return true after this, the entry is
 * in an undefined state.
 */
void list_del(volatile struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
}


/*
 * list_empty - tests whether a list is empty
 * @head: the list to test.
 */
 
// 测试一个链表是否为空 如果这个标的下一个指针指向的是自己，表示空了
_UBYTE list_empty(volatile struct list_head *head)
{
	return head->next == head;						// 双等号  判真假
}


/*
 * list_for_each	-	iterate over a list
 * @pos:	the &struct list_head to use as a loop cursor.
 * @head:	the head for your list.
 */
#define list_for_each(pos, head) \
	for( pos=(head)->next; pos!=(head); pos=pos->next )

/*
 * list_for_each	-	iterate over a list
 * @pos:	the &struct list_head to use as a loop cursor.
 * @head:	the head for your list.
 */
#define list_for_each_prev(pos, head) \
	for (pos = (head)->prev; pos != (head); pos = pos->prev)


#endif
