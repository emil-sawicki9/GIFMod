/****************************************************************************
** Meta object code from reading C++ file 'delegate.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../GUI/delegate.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'delegate.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Delegate_t {
    QByteArrayData data[8];
    char stringdata0[147];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Delegate_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Delegate_t qt_meta_stringdata_Delegate = {
    {
QT_MOC_LITERAL(0, 0, 8), // "Delegate"
QT_MOC_LITERAL(1, 9, 14), // "browserClicked"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 17), // "dirBrowserClicked"
QT_MOC_LITERAL(4, 43, 28), // "openParticleInitialCondition"
QT_MOC_LITERAL(5, 72, 31), // "openConstituentInitialCondition"
QT_MOC_LITERAL(6, 104, 29), // "openAqueousExchangeParameters"
QT_MOC_LITERAL(7, 134, 12) // "browserCheck"

    },
    "Delegate\0browserClicked\0\0dirBrowserClicked\0"
    "openParticleInitialCondition\0"
    "openConstituentInitialCondition\0"
    "openAqueousExchangeParameters\0"
    "browserCheck"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Delegate[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x0a /* Public */,
       3,    0,   45,    2, 0x0a /* Public */,
       4,    0,   46,    2, 0x0a /* Public */,
       5,    0,   47,    2, 0x0a /* Public */,
       6,    0,   48,    2, 0x0a /* Public */,
       7,    1,   49,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    2,

       0        // eod
};

void Delegate::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Delegate *_t = static_cast<Delegate *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->browserClicked(); break;
        case 1: _t->dirBrowserClicked(); break;
        case 2: _t->openParticleInitialCondition(); break;
        case 3: _t->openConstituentInitialCondition(); break;
        case 4: _t->openAqueousExchangeParameters(); break;
        case 5: _t->browserCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject Delegate::staticMetaObject = {
    { &QStyledItemDelegate::staticMetaObject, qt_meta_stringdata_Delegate.data,
      qt_meta_data_Delegate,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Delegate::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Delegate::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Delegate.stringdata0))
        return static_cast<void*>(const_cast< Delegate*>(this));
    return QStyledItemDelegate::qt_metacast(_clname);
}

int Delegate::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QStyledItemDelegate::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
